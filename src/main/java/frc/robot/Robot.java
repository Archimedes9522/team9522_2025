package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;
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
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 10.0;
  private static final double lowBatteryDisabledTime = 1.5;
  private boolean autoMessagePrinted;
  private boolean batteryNameWritten = false;
  private final Timer canErrorTimer = new Timer();
  private final Timer canErrorTimerInitial = new Timer();
  private final Timer disabledTimer = new Timer();

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);

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

    // Update CAN error alert
    var canStatus = RobotController.getCANStatus();
    if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
      canErrorTimer.reset();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }
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
  }

  private void updateSmartDashboard() {
    boolean isSetXButtonPressed = m_robotContainer.isSetXButtonPressed();
    SmartDashboard.putBoolean("SetX", isSetXButtonPressed);
    // Update PDH voltage
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
    SmartDashboard.putNumber("Robot Velocity", m_robotContainer.m_robotDrive.getRobotVelocity());
    m_field.setRobotPose(m_robotContainer.m_robotDrive.getPose());
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
