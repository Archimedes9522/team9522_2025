package frc.robot.subsystems.lift;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {
  // Enum for lift positions
  public enum LiftPosition {
    BOTTOM(LiftConstants.BOTTOM),
    LEVEL_ONE(LiftConstants.L_ONE),
    LEVEL_TWO(LiftConstants.L_TWO),
    LEVEL_THREE(LiftConstants.L_THREE),
    LEVEL_FOUR(LiftConstants.L_FOUR);

    private final double position;

    LiftPosition(double position) {
      this.position = position;
    }

    public double getPosition() {
      return position;
    }
  }

  private static final double POSITION_TOLERANCE = 0.5; // Position tolerance in inches

  private SparkMax leftMotor;
  private SparkMax rightMotor;
  private SparkClosedLoopController leftClosedLoopController;
  private SparkClosedLoopController rightClosedLoopController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private double currentSetpoint = 0.0;
  private LiftPosition currentTargetPosition = LiftPosition.BOTTOM;

  /**
   * Constructs the Lift subsystem and initializes hardware. Expects inches to be used for
   * setpoints.
   *
   * <p>Note: The left motor is inverted to work in opposition to the right motor.
   */
  public Lift() {
    // Initialize motors
    leftMotor = new SparkMax(LiftConstants.liftMotorOneCanID, MotorType.kBrushless);
    rightMotor = new SparkMax(LiftConstants.liftMotorTwoCanID, MotorType.kBrushless);

    // Configure motors - invert the left motor instead of negating its setpoint later
    configureMotor(leftMotor, true, true, LiftConstants.liftMotorTwoCanID);
    configureMotor(rightMotor, false, false, 0);

    // Get controllers and encoders after configuration
    leftClosedLoopController = leftMotor.getClosedLoopController();
    leftEncoder = leftMotor.getEncoder();
    rightClosedLoopController = rightMotor.getClosedLoopController();
    rightEncoder = rightMotor.getEncoder();
  }

  /**
   * Configures a SparkMax motor with standardized settings
   *
   * @param motor The SparkMax motor to configure
   * @param inverted Whether the motor direction should be inverted
   */
  private void configureMotor(SparkMax motor, boolean inverted, boolean follower, int followID) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure encoder
    motorConfig.encoder.positionConversionFactor(LiftConstants.POSITION_CONVERSION_FACTOR);

    // Configure closed loop controller with PID values
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(LiftConstants.PID_P)
        .i(LiftConstants.PID_I)
        .d(LiftConstants.PID_D)
        .outputRange(LiftConstants.OUTPUT_MIN, LiftConstants.OUTPUT_MAX);

    motorConfig.smartCurrentLimit(40);

    if (follower) {
      motorConfig.follow(followID, inverted);
    }

    // Set motor inversion if needed
    if (inverted) {
      motorConfig.inverted(inverted);
    }

    // Apply configuration
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Moves the lift to a predefined position.
   *
   * @param position The lift position to move to
   */
  public void runLiftToPos(LiftPosition position) {
    this.currentTargetPosition = position;
    this.currentSetpoint = position.getPosition();
    rightClosedLoopController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Moves the lift to a specific height in inches.
   *
   * @param position The position in inches
   */
  public void runLiftToPos(double position) {
    this.currentSetpoint = position;

    // Update the current target position if it matches one of our predefined positions
    for (LiftPosition liftPos : LiftPosition.values()) {
      if (Math.abs(position - liftPos.getPosition()) < 0.01) {
        this.currentTargetPosition = liftPos;
        break;
      }
    }

    rightClosedLoopController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /**
   * Gets the current target position.
   *
   * @return The current target position
   */
  public LiftPosition getCurrentTargetPosition() {
    return currentTargetPosition;
  }

  /**
   * Gets the current actual position in inches.
   *
   * @return The current position in inches
   */
  public double getCurrentPosition() {
    // Average the two encoders for a more reliable reading
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Checks if the lift is at its target position within tolerance.
   *
   * @return true if the lift is at the target position
   */
  public boolean isAtTargetPosition() {
    double currentAvgPosition = getCurrentPosition();
    return Math.abs(currentAvgPosition - currentSetpoint) < POSITION_TOLERANCE;
  }

  @Override
  public void periodic() {
    // Log motor currents
    Logger.recordOutput("Lift/LeftMotor/Current", leftMotor.getOutputCurrent());
    Logger.recordOutput("Lift/RightMotor/Current", rightMotor.getOutputCurrent());

    // Log motor positions
    Logger.recordOutput("Lift/LeftMotor/Position", leftEncoder.getPosition());
    Logger.recordOutput("Lift/RightMotor/Position", rightEncoder.getPosition());

    // Log motor velocities
    Logger.recordOutput("Lift/LeftMotor/Velocity", leftEncoder.getVelocity());
    Logger.recordOutput("Lift/RightMotor/Velocity", rightEncoder.getVelocity());

    // Log applied output
    Logger.recordOutput("Lift/LeftMotor/AppliedOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("Lift/RightMotor/AppliedOutput", rightMotor.getAppliedOutput());

    // Log temperatures
    Logger.recordOutput("Lift/LeftMotor/Temperature", leftMotor.getMotorTemperature());
    Logger.recordOutput("Lift/RightMotor/Temperature", rightMotor.getMotorTemperature());

    // Log control metrics
    Logger.recordOutput("Lift/Setpoint", currentSetpoint);
    Logger.recordOutput("Lift/LeftError", currentSetpoint - leftEncoder.getPosition());
    Logger.recordOutput("Lift/RightError", currentSetpoint - rightEncoder.getPosition());

    // Log motor synchronization (difference between motors)
    Logger.recordOutput(
        "Lift/PositionDifference", leftEncoder.getPosition() - rightEncoder.getPosition());

    // Additional logging
    Logger.recordOutput("Lift/IsAtPosition", isAtTargetPosition());
    Logger.recordOutput("Lift/CurrentPosition", getCurrentPosition());
    Logger.recordOutput("Lift/CurrentTargetPosition", currentTargetPosition.toString());
  }
}
