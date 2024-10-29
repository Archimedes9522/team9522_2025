package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  @Override
  public void robotInit() {
    initializeLogging();
    m_robotContainer = new RobotContainer();
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
    SmartDashboard.putData("PDH", m_pdh);
  }

  private void updateSmartDashboard() {
    boolean isSetXButtonPressed = m_robotContainer.isSetXButtonPressed();
    SmartDashboard.putBoolean("SetX", isSetXButtonPressed);
    // Update PDH voltage
    SmartDashboard.putNumber("PDH Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
  }

  private void initializeLogging() {
    Logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("Branch Name", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Commit Hash (Short)", BuildConstants.GIT_SHA.substring(0, 8));
    Logger.recordMetadata("Commit Hash (Full)", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);

    if (isReal()) {
      // Log to USB & Network Tables
      Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
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
