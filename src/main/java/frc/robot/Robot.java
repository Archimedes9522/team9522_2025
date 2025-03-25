package frc.robot;

import static frc.robot.Constants.VisionConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.json.simple.parser.ParseException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  // Autonomous command
  private Command m_autonomousCommand;

  // Sensors and hardware
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  private final PowerDistribution m_pdh = new PowerDistribution(1, ModuleType.kRev);

  // Robot container
  private RobotContainer m_robotContainer;

  // Field and vision
  private final Field2d m_field = new Field2d();
  private VisionSim visionSim;
  private PhotonCamera camera;
  private PhotonCamera camera1;
  private PhotonCamera camera2;
  private final double VISION_TURN_kP = 0.01;
  private final double VISION_DES_ANGLE_deg = 0.0;
  private final double VISION_STRAFE_kP = 0.5;
  private final double VISION_DES_RANGE_m = 1.25;

  // Autonomous names
  private String autoName, newAutoName;

  // Initialization method
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    camera = new PhotonCamera(VisionConstants.kFrontCam);
    camera1 = new PhotonCamera(VisionConstants.kLeftCam);
    camera2 = new PhotonCamera(VisionConstants.kRightCam);

    visionSim = new VisionSim(camera);
    setupSmartDashboard();
  }

  // Periodic method called every robot packet
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    updateSmartDashboard();
  }

  // Disabled mode initialization
  @Override
  public void disabledInit() {
    // Code for disabled initialization
  }

  // Disabled mode periodic method
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

  // Autonomous mode initialization
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // Autonomous mode periodic method
  @Override
  public void autonomousPeriodic() {
    // Code for autonomous periodic
  }

  // Teleoperated mode initialization
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Teleoperated mode periodic method
  @Override
  public void teleopPeriodic() {
    double forward = -m_robotContainer.m_driverController.getLeftY()
        * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double strafe = -m_robotContainer.m_driverController.getLeftX() * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    double turn = -m_robotContainer.m_driverController.getRightX() * Constants.DriveConstants.kMaxAngularSpeed;

    boolean targetVisible = false;
    double targetYaw = 0.0;
    double targetRange = 0.0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      var result = results.get(results.size() - 1);
      if (result.hasTargets()) {
        // At least one AprilTag was seen by the camera
        for (var target : result.getTargets()) {
          if (target.getFiducialId() == 7) {
            // Found Tag 7, record its information
            targetYaw = target.getYaw();
            targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                0.5, // Measured with a tape measure, or in CAD.
                1.435, // From 2024 game manual for ID 7
                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                Units.degreesToRadians(target.getPitch()));

            targetVisible = true;
          }
        }
      }
    }

    if (m_robotContainer.isVisionModeEnabled() && targetVisible) {
      turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.DriveConstants.kMaxAngularSpeed;
      forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP
          * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    }

    m_robotContainer.m_robotDrive.drive(forward, strafe, turn, true);

    // Put debug information to the dashboard
    SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    SmartDashboard.putNumber("Vision Target Range (m)", targetRange);
  }

  // Test mode initialization
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
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
  public void simulationPeriodic() {
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_robotContainer.getSimulationTotalCurrentDraw()));
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
}
