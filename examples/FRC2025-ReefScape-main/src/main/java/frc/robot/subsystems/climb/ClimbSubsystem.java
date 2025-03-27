// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

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
import frc.robot.Constants.ClimbConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that controls the climbing mechanism. It positions two claws that can grip the climbing
 * bar. Expects degrees to be used for setpoints.
 */
public class ClimbSubsystem extends SubsystemBase {
  // Motors
  private final SparkMax leftClawMotor;
  private final SparkMax rightClawMotor;

  // Controllers and encoders
  private SparkClosedLoopController leftPidController;
  private SparkClosedLoopController rightPidController;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  // Current state tracking
  private double currentSetpoint = 0.0;

  /** Constructs the ClimbSubsystem and initializes hardware */
  public ClimbSubsystem() {
    // Initialize motors
    leftClawMotor = new SparkMax(ClimbConstants.leftGrip_motorPort, MotorType.kBrushless);
    rightClawMotor = new SparkMax(ClimbConstants.rightGrip_motorPort, MotorType.kBrushless);

    // Configure motors - right motor needs to be inverted to work in opposition
    configureMotor(leftClawMotor, false);
    configureMotor(rightClawMotor, true);

    // Get controllers and encoders after configuration
    leftPidController = leftClawMotor.getClosedLoopController();
    leftEncoder = leftClawMotor.getEncoder();
    rightPidController = rightClawMotor.getClosedLoopController();
    rightEncoder = rightClawMotor.getEncoder();
  }

  /**
   * Configures a SparkMax motor with standardized settings
   *
   * @param motor The SparkMax motor to configure
   * @param inverted Whether the motor direction should be inverted
   */
  private void configureMotor(SparkMax motor, boolean inverted) {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    // Configure motor
    motorConfig.smartCurrentLimit(10).idleMode(IdleMode.kBrake);

    // Configure encoder
    // Position conversion factor converts from motor rotations to arm degrees
    // 49:1 gear ratio, 360 degrees in a full rotation
    motorConfig.encoder.positionConversionFactor(360.0 / ClimbConstants.CLAW_GEAR_RATIO);

    // Configure closed loop controller
    motorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ClimbConstants.CLAW_PID_P)
        .i(ClimbConstants.CLAW_PID_I)
        .d(ClimbConstants.CLAW_PID_D)
        .outputRange(ClimbConstants.CLAW_OUTPUT_MIN, ClimbConstants.CLAW_OUTPUT_MAX);

    // Set motor inversion if needed
    if (inverted) {
      motorConfig.inverted(inverted);
    }

    // Apply configuration
    motor.configure(
        motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Sets the position of the claws to a specific angle in degrees
   *
   * @param degrees The target position in degrees
   */
  public void setPosition(double degrees) {
    this.currentSetpoint = degrees;

    // Set both motors to the same target position (right motor is inverted in config)
    leftPidController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    rightPidController.setReference(
        this.currentSetpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  /** Move the claws to grip/closed position */
  public void moveToGripPosition() {
    setPosition(ClimbConstants.CLAW_GRIP_POSITION);
  }

  /** Move the claws to open position */
  public void moveToOpenPosition() {
    setPosition(ClimbConstants.CLAW_OPEN_POSITION);
  }

  /**
   * @return The current position of the claws in degrees (average of both encoders)
   */
  public double getCurrentPosition() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  /**
   * @return The current target position of the claws in degrees
   */
  public double getTargetPosition() {
    return currentSetpoint;
  }

  @Override
  public void periodic() {
    // Log motor currents
    Logger.recordOutput("Climb/LeftMotor/Current", leftClawMotor.getOutputCurrent());
    Logger.recordOutput("Climb/RightMotor/Current", rightClawMotor.getOutputCurrent());

    // Log motor positions
    Logger.recordOutput("Climb/LeftMotor/Position", leftEncoder.getPosition());
    Logger.recordOutput("Climb/RightMotor/Position", rightEncoder.getPosition());
    Logger.recordOutput("Climb/AveragePosition", getCurrentPosition());

    // Log motor velocities
    Logger.recordOutput("Climb/LeftMotor/Velocity", leftEncoder.getVelocity());
    Logger.recordOutput("Climb/RightMotor/Velocity", rightEncoder.getVelocity());

    // Log applied output
    Logger.recordOutput("Climb/LeftMotor/AppliedOutput", leftClawMotor.getAppliedOutput());
    Logger.recordOutput("Climb/RightMotor/AppliedOutput", rightClawMotor.getAppliedOutput());

    // Log temperatures
    Logger.recordOutput("Climb/LeftMotor/Temperature", leftClawMotor.getMotorTemperature());
    Logger.recordOutput("Climb/RightMotor/Temperature", rightClawMotor.getMotorTemperature());

    // Log control metrics
    Logger.recordOutput("Climb/Setpoint", currentSetpoint);
    Logger.recordOutput("Climb/LeftError", currentSetpoint - leftEncoder.getPosition());
    Logger.recordOutput("Climb/RightError", currentSetpoint - rightEncoder.getPosition());

    // Log motor synchronization (difference between motors)
    Logger.recordOutput(
        "Climb/PositionDifference", leftEncoder.getPosition() - rightEncoder.getPosition());
  }
}
