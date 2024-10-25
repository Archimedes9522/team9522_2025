package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private final Field2d m_field = new Field2d();
    private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
        SmartDashboard.putData("Field", m_field);
        setupSmartDashboard();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        updateSmartDashboard();
    }

    @Override
    public void autonomousInit() {
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();

      if (m_autonomousCommand != null) {
       m_autonomousCommand.schedule();
      }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    public void setupSmartDashboard() {
        DriveSubsystem driveSubsystem = m_robotContainer.getDriveSubsystem();
        SmartDashboard.putData("Swerve Drive", createSwerveDriveSendable(driveSubsystem));
        SmartDashboard.putData("PDH", m_pdh);
    }

    private void updateSmartDashboard() {
        boolean isSetXButtonPressed = m_robotContainer.getDriverController().getRawButton(Constants.kSetXButton);
        SmartDashboard.putBoolean("SetX", isSetXButtonPressed);
        // Update PDH voltage
        SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
        // Update robot velocity
        DriveSubsystem driveSubsystem = m_robotContainer.getDriveSubsystem();
        double robotVelocity = driveSubsystem.getRobotVelocity();
        SmartDashboard.putNumber("Robot Velocity", robotVelocity);
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());

        Pose2d currentPose = driveSubsystem.getPose();
        double transformedX = -currentPose.getY();
        double transformedY = -currentPose.getX();
        Rotation2d transformedRotation = currentPose.getRotation();
        Pose2d transformedPose = new Pose2d(transformedX, transformedY, transformedRotation);
        m_field.setRobotPose(transformedPose);
    }

    private Sendable createSwerveDriveSendable(DriveSubsystem driveSubsystem) {
        return new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");
                for (int i = 0; i < 4; i++) {
                    final int index = i;
                    builder.addDoubleProperty("Module " + index + " Angle", 
                        () -> driveSubsystem.getModuleStates()[index].angle.getRadians(), null);
                    builder.addDoubleProperty("Module " + index + " Velocity", 
                        () -> driveSubsystem.getModuleStates()[index].speedMetersPerSecond, null);
                }
            }
        };
    }
}