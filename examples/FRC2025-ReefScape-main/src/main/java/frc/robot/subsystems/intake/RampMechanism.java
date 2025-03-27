package frc.robot.subsystems.intake;

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
import frc.robot.Constants.RampMechanismConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the ramp/arm mechanism This allows the robot to position the ramp at
 * various angles for intake, outtake, or hang operations. Expects degrees to be used for setpoints.
 */
public class RampMechanism extends SubsystemBase {
  // Motors
  private final SparkMax leftPivotMotor;
  private final SparkMax rightPivotMotor;

  // Controllers and encoders
  private SparkClosedLoopController leftPidController;
  private SparkClosedLoopController rightPidController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  // Current state tracking
  private double currentSetpoint = 0.0;

  /** Constructs the RampMechanism subsystem and initializes hardware */
  public RampMechanism() {
    // Initialize motors
    leftPivotMotor =
        new SparkMax(RampMechanismConstants.pivotMotorOne_motorPort, MotorType.kBrushless);
    rightPivotMotor =
        new SparkMax(RampMechanismConstants.pivotMotorTwo_motorPort, MotorType.kBrushless);

    // Configure motors - right motor needs to be inverted to work in opposition
    configureMotor(leftPivotMotor, true, true, RampMechanismConstants.pivotMotorTwo_motorPort);
    configureMotor(rightPivotMotor, false, false, 0);

    // Get controllers and encoders after configuration
    leftPidController = leftPivotMotor.getClosedLoopController();
    leftEncoder = leftPivotMotor.getEncoder();
    rightPidController = rightPivotMotor.getClosedLoopController();
    rightEncoder = rightPivotMotor.getEncoder();
  }

  /**
   * Configures a SparkMax motor with standardized settings
   *
   * @param motor The SparkMax motor to configure
   * @param inverted Whether the motor direction should be inverted
   */
  private void configureMotor(SparkMax motor, boolean inverted, boolean follower, int followerID) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure motor
    motorConfig
        .smartCurrentLimit(RampMechanismConstants.rampMechanismPivot_stallAmperage)
        .idleMode(IdleMode.kBrake);

    // Configure encoder
    motorConfig.encoder.positionConversionFactor(
        360.0 / RampMechanismConstants.rampMechanismPivot_gearRatio);

    // Configure closed loop controller
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(RampMechanismConstants.rampMechanismPivot_kP)
        .i(RampMechanismConstants.rampMechanismPivot_kI)
        .d(RampMechanismConstants.rampMechanismPivot_kD)
        .outputRange(RampMechanismConstants.OUTPUT_MIN, RampMechanismConstants.OUTPUT_MAX);

    if (follower) {
      motorConfig.follow(followerID, inverted);
    }

    // Set motor inversion if needed
    if (inverted) {
      motorConfig.inverted(inverted);
    }

    // Apply configuration
    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Rotates the ramp mechanism to a specific position in degrees
   *
   * @param degrees The target position in degrees
   */
  public void setPosition(double degrees) {
    this.currentSetpoint = degrees;

    rightPidController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Move the ramp to intake position */
  public void moveToIntakePosition() {
    setPosition(RampMechanismConstants.ROTATION_INTAKE);
  }

  /** Move the ramp to hang position */
  public void moveToHangPosition() {
    setPosition(RampMechanismConstants.ROTATION_HANG);
  }

  /** Move the ramp to hang position */
  public void pullCageDown() {
    setPosition(RampMechanismConstants.PULL_CAGE_DOWN);
  }

  /**
   * @return The current position of the ramp in degrees (average of both encoders)
   */
  public double getCurrentPosition() {
    return (leftEncoder.getPosition() - rightEncoder.getPosition()) / 2.0;
  }

  /**
   * @return The current target position of the ramp in degrees
   */
  public double getTargetPosition() {
    return currentSetpoint;
  }

  @Override
  public void periodic() {
    // Log motor currents
    Logger.recordOutput("RampMechanism/LeftMotor/Current", leftPivotMotor.getOutputCurrent());
    Logger.recordOutput("RampMechanism/RightMotor/Current", rightPivotMotor.getOutputCurrent());

    // Log motor positions
    Logger.recordOutput("RampMechanism/LeftMotor/Position", leftEncoder.getPosition());
    Logger.recordOutput("RampMechanism/RightMotor/Position", rightEncoder.getPosition());
    Logger.recordOutput("RampMechanism/AveragePosition", getCurrentPosition());

    // Log motor velocities
    Logger.recordOutput("RampMechanism/LeftMotor/Velocity", leftEncoder.getVelocity());
    Logger.recordOutput("RampMechanism/RightMotor/Velocity", rightEncoder.getVelocity());

    // Log applied output
    Logger.recordOutput("RampMechanism/LeftMotor/AppliedOutput", leftPivotMotor.getAppliedOutput());
    Logger.recordOutput(
        "RampMechanism/RightMotor/AppliedOutput", rightPivotMotor.getAppliedOutput());

    // Log temperatures
    Logger.recordOutput(
        "RampMechanism/LeftMotor/Temperature", leftPivotMotor.getMotorTemperature());
    Logger.recordOutput(
        "RampMechanism/RightMotor/Temperature", rightPivotMotor.getMotorTemperature());

    // Log control metrics
    Logger.recordOutput("RampMechanism/Setpoint", currentSetpoint);
    Logger.recordOutput("RampMechanism/LeftError", currentSetpoint - leftEncoder.getPosition());
    Logger.recordOutput("RampMechanism/RightError", currentSetpoint - rightEncoder.getPosition());

    // Log motor synchronization (difference between motors)
    Logger.recordOutput(
        "RampMechanism/PositionDifference", leftEncoder.getPosition() - rightEncoder.getPosition());
  }
}
