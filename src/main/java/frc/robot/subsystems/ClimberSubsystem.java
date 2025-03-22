package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberSubsystemConstants;
import frc.robot.Constants.ClimberSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.SimulationRobotConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax climberMotor;
    private final SparkClosedLoopController climberController;
    private final RelativeEncoder climberEncoder;

    private double currentSetpoint;
    private boolean isInside;
    private boolean wasResetByButton = false;

    // Simulation related objects
    private final DCMotor climberMotorModel = DCMotor.getNeo550(1);
    private SparkMaxSim climberMotorSim;
    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
            climberMotorModel,
            ClimberSubsystemConstants.kClimberGearReduction,
            SingleJointedArmSim.estimateMOI(0.5, 5.0),
            0.5,
            Units.degreesToRadians(0),
            Units.degreesToRadians(90),
            true,
            0);

    // Visualization for the mechanism
    private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
    private final MechanismRoot2d m_mechRoot = m_mech2d.getRoot("Climber Root", 25, 0);
    private final MechanismLigament2d m_climberArmMech = m_mechRoot.append(
            new MechanismLigament2d(
                    "Climber Arm",
                    0.5 * SimulationRobotConstants.kPixelsPerMeter,
                    90));

    public ClimberSubsystem() {
        climberMotor = new SparkMax(ClimberSubsystemConstants.kClimberMotorCanId, MotorType.kBrushless);
        climberController = climberMotor.getClosedLoopController();
        climberEncoder = climberMotor.getEncoder();

        // Configure the climber motor with settings from Configs
        climberMotor.configure(
                Configs.ClimberSubsystem.climberConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Start with arm inside
        currentSetpoint = ArmSetpoints.kInside;
        isInside = true;

        // Initialize simulation objects
        climberMotorSim = new SparkMaxSim(climberMotor, climberMotorModel);

        // Add mechanism to dashboard
        SmartDashboard.putData("Climber Subsystem", m_mech2d);

        // Zero encoder on initialization
        climberEncoder.setPosition(0);
    }

    /**
     * Drive the climber motor to its setpoint using MAXMotion position control
     * for smooth acceleration and deceleration.
     */
    private void moveToSetpoint() {
        climberController.setReference(currentSetpoint, ControlType.kMAXMotionPositionControl);
    }

    /**
     * Zero the encoder when the user button is pressed on the roboRIO.
     */
    private void zeroOnUserButton() {
        if (!wasResetByButton && RobotController.getUserButton()) {
            // Zero the encoder only when button switches from "unpressed" to "pressed"
            wasResetByButton = true;
            climberEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasResetByButton = false;
        }
    }

    public Command setArmPositionCommand(double position) {
        return this.runOnce(() -> {
            currentSetpoint = position;
            isInside = (position == ArmSetpoints.kInside);
        }).withName("SetClimberPosition");
    }

    public Command moveToInsideCommand() {
        return setArmPositionCommand(ArmSetpoints.kInside);
    }

    public Command moveToOutsideCommand() {
        return setArmPositionCommand(ArmSetpoints.kOutside);
    }

    public Command togglePositionCommand() {
        return this.runOnce(() -> {
            if (isInside) {
                currentSetpoint = ArmSetpoints.kOutside;
                isInside = false;
            } else {
                currentSetpoint = ArmSetpoints.kInside;
                isInside = true;
            }
        }).withName("ToggleClimberPosition");
    }

    /**
     * Command to run the climber up for a specific duration.
     * Useful for autonomous sequences.
     */
    public Command runClimberUpCommand(double seconds) {
        return Commands.sequence(
                Commands.runOnce(() -> climberMotor.set(ClimberSubsystemConstants.kClimberSpeed)),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> climberMotor.set(0))).withName("RunClimberUp");
    }

    /**
     * Command to run the climber down for a specific duration.
     * Useful for autonomous sequences.
     */
    public Command runClimberDownCommand(double seconds) {
        return Commands.sequence(
                Commands.runOnce(() -> climberMotor.set(ClimberSubsystemConstants.kClimberDownSpeed)),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> climberMotor.set(0))).withName("RunClimberDown");
    }

    public boolean isInside() {
        return isInside;
    }

    @Override
    public void periodic() {
        // Drive to setpoint using closed-loop control
        moveToSetpoint();
        zeroOnUserButton();

        // Update dashboard
        SmartDashboard.putNumber("Climber/Target Position", currentSetpoint);
        SmartDashboard.putNumber("Climber/Current Position", climberEncoder.getPosition());
        SmartDashboard.putBoolean("Climber/Is Inside", isInside);
        SmartDashboard.putNumber("Climber/Applied Output", climberMotor.getAppliedOutput());

        // Update mechanism visualization
        double angle = Units.rotationsToDegrees(
                climberEncoder.getPosition() / ClimberSubsystemConstants.kClimberGearReduction);
        m_climberArmMech.setAngle(angle);
    }

    @Override
    public void simulationPeriodic() {
        // In this method, we update our simulation of what our climber is doing
        // First, we set our "inputs" (voltages)
        m_armSim.setInput(climberMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

        // Next, we update it. The standard loop time is 20ms.
        m_armSim.update(0.020);

        // Iterate the climber SPARK simulation
        climberMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(
                        m_armSim.getVelocityRadPerSec() * ClimberSubsystemConstants.kClimberGearReduction),
                RobotController.getBatteryVoltage(),
                0.02);

        // Update the position from simulation
        climberMotorSim.setPosition(
                m_armSim.getAngleRads() * ClimberSubsystemConstants.kClimberGearReduction / (2 * Math.PI));
    }

    /** Get the current drawn by the simulation physics model */
    public double getSimulationCurrentDraw() {
        return m_armSim.getCurrentDrawAmps();
    }
}