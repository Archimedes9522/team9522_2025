package frc.robot.subsystems.endEffector;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem for the end effector with intake wheels and limit switches. This subsystem provides
 * direct control of the motors and access to sensor states.
 */
public class EndEffector extends SubsystemBase {
  // Motors
  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;
  private final SparkMax algaeRemoveMotor;

  private SparkClosedLoopController algaePidController;
  private RelativeEncoder algaeEncoder;

  // Current state tracking
  private double currentSetpoint = 0.0;
  private boolean isRemovingPos = false;

  // Limit switches (May need this: https://github.com/PeterJohnson/rpi-colorsensor)
  // private final ColorSensorV3 frontSensor;
  private final ColorSensorV3 backSensor;

  // Current state tracking
  private IntakeState currentState = IntakeState.STOPPED;

  /** Possible states of the intake motors */
  public enum IntakeState {
    FORWARD, // Moving game piece from back to front (normal operation)
    REVERSE, // Rarely used - moves game piece backward
    STOPPED
  }

  /** Constructs the EndEffector subsystem and initializes hardware */
  public EndEffector() {
    // Initialize motors
    leftIntakeMotor =
        new SparkMax(EndEffectorConstants.endEffectorMotorOne_Port, MotorType.kBrushless);
    rightIntakeMotor =
        new SparkMax(EndEffectorConstants.endEffectorMotorTwo_Port, MotorType.kBrushless);
    algaeRemoveMotor =
        new SparkMax(EndEffectorConstants.algeDescorerMotor_Port, MotorType.kBrushless);

    // Configure motors
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake).inverted(false);
    leftIntakeMotor.configure(
        leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake).inverted(true);
    rightIntakeMotor.configure(
        rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig algaeConfig = new SparkMaxConfig();
    algaeConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake).inverted(true);

    // Configure encoder
    // Position conversion factor converts from motor rotations to arm degrees
    // 56.8:1 gear ratio, 360 degrees in a full rotation
    algaeConfig.encoder.positionConversionFactor(360.0 / 56.8);

    // Configure closed loop controller
    algaeConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0075) // 0.01
        .i(0) // 0
        .d(0.01) // 0.01
        .outputRange(-0.75, 0.8);

    algaeRemoveMotor.configure(
        algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Get controllers and encoders after configuration
    algaePidController = algaeRemoveMotor.getClosedLoopController();
    algaeEncoder = algaeRemoveMotor.getEncoder();

    // Initialize limit switches
    // frontSensor = new ColorSensorV3(EndEffectorConstants.i2cPortRio); // Under 70 Normal
    backSensor = new ColorSensorV3(EndEffectorConstants.i2cPortNavX); // Under 110 Normal
  }

  /**
   * Runs motors to move game pieces from back to front (standard operation)
   *
   * @param speedFactor A multiplier for the base speed (0.0-1.0)
   */
  public void runMotors(double speedFactor) {
    double speed = EndEffectorConstants.endEffectorSpeed * speedFactor;
    leftIntakeMotor.set(-speed); // Negative for outward motion (moves piece forward)
    rightIntakeMotor.set(-speed);
    currentState = IntakeState.FORWARD;
  }

  /** Runs motors at default speed to move game pieces from back to front */
  public void runMotors() {
    runMotors(1.0);
  }

  /**
   * RARELY USED: Reverses motor direction (moves game piece backward) Only use in emergency
   * situations where a piece is stuck
   *
   * @param speedFactor A multiplier for the base speed (0.0-1.0)
   */
  public void reverseMotors(double speedFactor) {
    double speed = EndEffectorConstants.endEffectorSpeed * speedFactor;
    leftIntakeMotor.set(speed); // Positive for inward motion
    rightIntakeMotor.set(speed);
    currentState = IntakeState.REVERSE;
  }

  /**
   * Sets the speed of the algae remover motor
   *
   * @param speed Speed from -1.0 to 1.0
   */
  public void setAlgaeMotorSpeed(double speed) {
    algaeRemoveMotor.set(speed);
  }

  /**
   * Sets the position of the thingy to a specific angle in degrees
   *
   * @param degrees The target position in degrees
   */
  public void setPosition(double degrees) {
    this.currentSetpoint = degrees;

    // Set both motors to the same target position (right motor is inverted in config)
    algaePidController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Move the claws to grip/closed position */
  public void moveToRemovePos() {
    setPosition(-55);
  }

  /** Move the claws to open position */
  public void moveToStow() {
    setPosition(-5);
  }

  public void moveToReadyPos() {
    setPosition(-200);
  }

  public void toggleAlgaePos() {
    if (isRemovingPos) {
      moveToStow();
    } else {
      moveToRemovePos();
    }

    // Set for future;
    isRemovingPos = !isRemovingPos;
  }

  /** Stops all motors */
  public void stop() {
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
    currentState = IntakeState.STOPPED;
  }

  /**
   * @return true if the intake limit switch is triggered (game piece detected at intake position)
   */
  // public boolean isFrontSensorTriggered() {
  //   return frontSensor.getProximity() > EndEffectorConstants.FRONT_PROX_TRIGGER;
  //   // return false;
  // }

  /**
   * @return true if the outtake limit switch is triggered (game piece detected at outtake position)
   */
  public boolean isBackSensorTriggered() {
    return backSensor.getProximity() > EndEffectorConstants.BACK_PROX_TRIGGER;
  }

  /**
   * @return the current state of the intake motors
   */
  public IntakeState getCurrentState() {
    return currentState;
  }

  /**
   * Gets the current draw of the algae remover motor
   *
   * @return Current in amps
   */
  public double getAlgaeMotorCurrent() {
    return algaeRemoveMotor.getOutputCurrent();
  }

  /**
   * Gets the current position of the algae remover
   *
   * @return Position in degrees
   */
  public double getAlgaePosition() {
    return algaeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // Log sensor values and state
    // Logger.recordOutput("EndEffector/frontSensor", isFrontSensorTriggered());
    Logger.recordOutput("EndEffector/backSensor", isBackSensorTriggered());
    // Logger.recordOutput("EndEffector/frontSensorRawIR", frontSensor.getIR());
    Logger.recordOutput("EndEffector/backSensorRawIR", backSensor.getIR());
    // Logger.recordOutput("EndEffector/frontSensorRawColor", frontSensor.getRawColor().toString());
    Logger.recordOutput("EndEffector/backSensorRawColor", backSensor.getRawColor().toString());
    // Logger.recordOutput("EndEffector/frontSensorRawProximity", frontSensor.getProximity());
    Logger.recordOutput("EndEffector/backSensorRawProximity", backSensor.getProximity());
    Logger.recordOutput("EndEffector/backSensorConnected", backSensor.isConnected());
    // Logger.recordOutput("EndEffector/frontSensorConnected", frontSensor.isConnected());
    Logger.recordOutput("EndEffector/State", currentState.toString());
    Logger.recordOutput("EndEffector/LeftMotor/Output", leftIntakeMotor.getAppliedOutput());
    Logger.recordOutput("EndEffector/RightMotor/Output", rightIntakeMotor.getAppliedOutput());
    Logger.recordOutput("EndEffector/algaeMotor/Output", algaeRemoveMotor.getAppliedOutput());
    Logger.recordOutput("EndEffector/algaeMotor/Current", algaeRemoveMotor.getOutputCurrent());
    Logger.recordOutput("EndEffector/algaeMotor/Setpoint", currentSetpoint);
    Logger.recordOutput("EndEffector/algaeMotor/CurrentPos", algaeEncoder.getPosition());
    Logger.recordOutput("EndEffector/LeftMotor/Current", leftIntakeMotor.getOutputCurrent());
    Logger.recordOutput("EndEffector/RightMotor/Current", rightIntakeMotor.getOutputCurrent());
  }
}
