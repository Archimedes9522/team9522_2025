package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

public class PhotonDriveSubsystem extends SubsystemBase {
    // The drive subsystem (could be any drivetrain implementation)
    private final DriveSubsystem m_driveSubsystem;

    // PhotonVision cameras
    private final PhotonCamera m_driverCamera;
    private final PhotonCamera m_leftCamera;
    private final PhotonCamera m_rightCamera;

    // PID controllers for vision alignment
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotController;

    // Tracking status
    private boolean m_hasTarget = false;
    private PhotonTrackedTarget m_bestTarget = null;
    private int m_lastSeenCameraIndex = -1; // -1 = none, 0 = left, 1 = right

    public PhotonDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        // Initialize cameras
        m_driverCamera = new PhotonCamera(VisionConstants.kDriverCameraName);
        m_leftCamera = new PhotonCamera(VisionConstants.kLeftCameraName);
        m_rightCamera = new PhotonCamera(VisionConstants.kRightCameraName);

        // Configure driver camera for driver view (if needed)
        if (VisionConstants.kConfigureDriverCameraForDriving) {
            m_driverCamera.setDriverMode(true);
        }

        // Configure PID controllers
        m_xController = new PIDController(VisionConstants.kPX, VisionConstants.kIX, VisionConstants.kDX);
        m_yController = new PIDController(VisionConstants.kPY, VisionConstants.kIY, VisionConstants.kDY);
        m_rotController = new PIDController(VisionConstants.kPRot, VisionConstants.kIRot, VisionConstants.kDRot);
        m_rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        updateVision();

        // Display target information to SmartDashboard
        SmartDashboard.putBoolean("Vision/HasTarget", m_hasTarget);
        if (m_hasTarget && m_bestTarget != null) {
            SmartDashboard.putNumber("Vision/TargetID", m_bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/TargetYaw", m_bestTarget.getYaw());
            SmartDashboard.putNumber("Vision/TargetPitch", m_bestTarget.getPitch());
            SmartDashboard.putNumber("Vision/LastCamera", m_lastSeenCameraIndex);
        }
    }

    /**
     * Updates vision data from both cameras and selects the best target
     */
    private void updateVision() {
        // Get results from both cameras
        PhotonPipelineResult leftResult = m_leftCamera.getLatestResult();
        PhotonPipelineResult rightResult = m_rightCamera.getLatestResult();

        // Check if either camera sees a target
        boolean leftHasTargets = leftResult.hasTargets();
        boolean rightHasTargets = rightResult.hasTargets();

        m_hasTarget = leftHasTargets || rightHasTargets;

        if (!m_hasTarget) {
            // No targets visible
            m_bestTarget = null;
            return;
        }

        // Collect all targets from both cameras
        List<CameraTarget> allTargets = new ArrayList<>();

        if (leftHasTargets) {
            for (PhotonTrackedTarget target : leftResult.getTargets()) {
                allTargets.add(new CameraTarget(target, 0)); // 0 = left camera
            }
        }

        if (rightHasTargets) {
            for (PhotonTrackedTarget target : rightResult.getTargets()) {
                allTargets.add(new CameraTarget(target, 1)); // 1 = right camera
            }
        }

        // Select the best target based on criteria (closest to center, largest, etc.)
        Optional<CameraTarget> bestTarget = allTargets.stream()
                .min(Comparator.comparing(target -> Math.abs(target.target.getYaw())));

        if (bestTarget.isPresent()) {
            m_bestTarget = bestTarget.get().target;
            m_lastSeenCameraIndex = bestTarget.get().cameraIndex;
        }
    }

    /**
     * @return True if either camera currently sees any AprilTags
     */
    public boolean hasTarget() {
        return m_hasTarget;
    }

    /**
     * Get the ID of the currently tracked AprilTag
     * 
     * @return The ID of the currently tracked tag, or -1 if no tag is visible
     */
    public int getTargetId() {
        if (m_hasTarget && m_bestTarget != null) {
            return m_bestTarget.getFiducialId();
        }
        return -1;
    }

    /**
     * Get the camera that last saw the target
     * 
     * @return 0 for left camera, 1 for right camera, -1 if no target seen
     */
    public int getLastSeenCamera() {
        return m_lastSeenCameraIndex;
    }

    /**
     * Drive method that aligns to the currently detected AprilTag
     * 
     * @param manualX            Optional manual X input (-1.0 to 1.0)
     * @param manualY            Optional manual Y input (-1.0 to 1.0)
     * @param manualRot          Optional manual rotation input (-1.0 to 1.0)
     * @param useVisionAlignment If true, will use vision to align to target
     */
    public void driveWithVisionAlignment(double manualX, double manualY, double manualRot, boolean useVisionAlignment) {
        if (useVisionAlignment && hasTarget() && m_bestTarget != null) {
            // Calculate vision-based drive corrections
            double yaw = m_bestTarget.getYaw();
            double pitch = m_bestTarget.getPitch();

            // Apply camera offset correction if using left/right camera
            if (m_lastSeenCameraIndex >= 0) {
                double[] cameraYawOffset = {
                        VisionConstants.kLeftCameraYawOffset,
                        VisionConstants.kRightCameraYawOffset
                };
                yaw += cameraYawOffset[m_lastSeenCameraIndex];
            }

            // Calculate control outputs
            double xSpeed = -m_xController.calculate(pitch, 0);
            double ySpeed = -m_yController.calculate(yaw, 0);
            double rotSpeed = -m_rotController.calculate(m_bestTarget.getSkew(), 0);

            // Clamp speeds to prevent too aggressive movement
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), DriveConstants.kMaxSpeedMetersPerSecond), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), DriveConstants.kMaxSpeedMetersPerSecond), ySpeed);
            rotSpeed = Math.copySign(Math.min(Math.abs(rotSpeed), DriveConstants.kMaxAngularSpeedRadiansPerSecond),
                    rotSpeed);

            // Drive with calculated speeds
            m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
        } else {
            // No target or vision alignment not requested, use manual controls
            double xSpeed = manualX * DriveConstants.kMaxSpeedMetersPerSecond;
            double ySpeed = manualY * DriveConstants.kMaxSpeedMetersPerSecond;
            double rotSpeed = manualRot * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

            m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
        }
    }

    /**
     * Helper class to keep track of which camera saw a target
     */
    private static class CameraTarget {
        public PhotonTrackedTarget target;
        public int cameraIndex; // 0 = left, 1 = right

        public CameraTarget(PhotonTrackedTarget target, int cameraIndex) {
            this.target = target;
            this.cameraIndex = cameraIndex;
        }
    }
}