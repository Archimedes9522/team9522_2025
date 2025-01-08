package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();
  public String autoName, newAutoName;
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  @Override
  public void robotInit() {
    initializeLogging();
    m_robotContainer = new RobotContainer();
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
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("PDH", m_pdh);
    SmartDashboard.putBoolean("Reset Pose", false);
  }

  private void updateSmartDashboard() {
    boolean isSetXButtonPressed = m_robotContainer.isSetXButtonPressed();
    SmartDashboard.putBoolean("SetX", isSetXButtonPressed);
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("Robot Velocity", m_robotContainer.m_robotDrive.getRobotVelocity());

    // Update both Field2d and Swerve widget with the same pose
    var robotPose = m_robotContainer.m_robotDrive.getPose();
    m_field.setRobotPose(robotPose);

    // Keep rotation consistent with Field2d widget
    SmartDashboard.putNumber("Swerve/Robot X", robotPose.getX());
    SmartDashboard.putNumber("Swerve/Robot Y", robotPose.getY());
    SmartDashboard.putNumber("Swerve/Robot Angle", robotPose.getRotation().getRadians());

    // Handle pose reset button
    if (SmartDashboard.getBoolean("Reset Pose", false)) {
      m_robotContainer.m_robotDrive.resetPose();
      SmartDashboard.putBoolean("Reset Pose", false); // Reset the button
    }
  }

  @Override
  public void disabledPeriodic() {
    /*newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName) {
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        System.out.println("Displaying " + autoName);
        List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : pathPlannerPaths) {
          poses.addAll(
              path.getAllPathPoints().stream()
                  .map(
                      point ->
                          new Pose2d(
                              point.position.getX(), point.position.getY(), new Rotation2d()))
                  .collect(Collectors.toList()));
        }
        m_field.getObject("path").setPoses(poses);
      }
    }*/
  }

  private void initializeLogging() {
    Logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Branch Name", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Commit Hash (Short)", BuildConstants.GIT_SHA.substring(0, 8));
    Logger.recordMetadata("Commit Hash (Full)", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);

    if (isReal()) {
      // Log to USB & Network Tables
      Logger.addDataReceiver(new NT4Publisher());
    } else if (isSimulation()) {
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      // Replay from log and save to file
      setUseTiming(false);
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }
    Logger.registerURCL(URCL.startExternal());
    Logger.start();
  }
}
