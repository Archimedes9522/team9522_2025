package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private RobotContainer m_robotContainer;
  private final Field2d m_field = new Field2d();
  private String autoName, newAutoName;
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    setupSmartDashboard();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    updateSmartDashboard();
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  public void setupSmartDashboard() {
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("PDH", m_pdh);
    SmartDashboard.putBoolean("Reset Pose", false);
    SmartDashboard.putBoolean("SetX", false);
    SmartDashboard.putNumber("Gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro roll", m_gyro.getRoll());
  }

  private void updateSmartDashboard() {
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("Robot Velocity", m_robotContainer.m_robotDrive.getRobotVelocity());

    // Update both Field2d and Swerve widget with the same pose
    var robotPose = m_robotContainer.m_robotDrive.getPose();
    m_field.setRobotPose(robotPose);

    // Handle pose reset button
    if (SmartDashboard.getBoolean("Reset Pose", false)) {
      m_robotContainer.m_robotDrive.zeroHeadingCommand();
      m_robotContainer.m_robotDrive.resetOdometry(new Pose2d());
      SmartDashboard.putBoolean("Reset Pose", false); // Reset the button
    }

    if (SmartDashboard.getBoolean("SetX", false)) {
      m_robotContainer.m_robotDrive.setXCommand();
      SmartDashboard.putBoolean("SetX", false);
    }

    SmartDashboard.putNumber("Gyro angle", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro pitch", m_gyro.getPitch());
    SmartDashboard.putNumber("Gyro roll", m_gyro.getRoll());
  }

  @Override
  public void disabledPeriodic() {
    newAutoName = m_robotContainer.getAutonomousCommand().getName();
    if (autoName != newAutoName) {
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
        System.out.println("Displaying " + autoName);
        try {
          List<PathPlannerPath> pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
            poses.addAll(
                path.getAllPathPoints().stream()
                    .map(
                        point -> new Pose2d(
                            point.position.getX(), point.position.getY(), new Rotation2d()))
                    .collect(Collectors.toList()));
          }
          m_field.getObject("path").setPoses(poses);
        } catch (IOException | ParseException e) {
          e.printStackTrace();
        }
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_robotContainer.getSimulationTotalCurrentDraw()));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
}
