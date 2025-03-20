package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

public class PhotonDriveSubsystem extends SubsystemBase {
    // The drive subsystem
    private final DriveSubsystem m_driveSubsystem;

    // PhotonVision cameras
    private final PhotonCamera m_driverCamera;
    private final PhotonCamera m_leftCamera;
    private final PhotonCamera m_rightCamera;

    // PID controllers for vision alignment
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotController;

    // Target tracking variables
    private boolean m_hasTarget = false;
    private PhotonTrackedTarget m_bestTarget = null;
    private int m_lastSeenCameraIndex = -1; // -1 = none, 0 = left, 1 = right

    /**
     * Creates a new PhotonDriveSubsystem
     * 
     * @param driveSubsystem The drive subsystem to control
     */
    public PhotonDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        // Initialize cameras
        m_driverCamera = new PhotonCamera(VisionConstants.kDriverCameraName);
        m_leftCamera = new PhotonCamera(VisionConstants.kLeftCameraName);
        m_rightCamera = new PhotonCamera(VisionConstants.kRightCameraName);

        // Configure cameras appropriately
        m_driverCamera.setDriverMode(true); // Always set driver camera to driver mode

        // Set AprilTag cameras to appropriate pipeline
        try {
            m_leftCamera.setPipelineIndex(0); // Assuming pipeline 0 is for AprilTags
            m_rightCamera.setPipelineIndex(0); // Assuming pipeline 0 is for AprilTags
        } catch (Exception e) {
            // Log error if pipelines couldn't be set
            System.err.println("Error setting camera pipelines: " + e.getMessage());
        }

        // Configure PID controllers for vision alignment
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
        SmartDashboard.putBoolean("Vision/LeftCameraConnected", m_leftCamera.isConnected());
        SmartDashboard.putBoolean("Vision/RightCameraConnected", m_rightCamera.isConnected());

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
        try {
            // Get latest results from both cameras
            PhotonPipelineResult leftResult = m_leftCamera.getLatestResult();
            PhotonPipelineResult rightResult = m_rightCamera.getLatestResult();

            // Find the best target from either camera
            if (leftResult.hasTargets() && rightResult.hasTargets()) {
                // If both cameras see targets, use the one with the largest target
                PhotonTrackedTarget leftBest = leftResult.getBestTarget();
                PhotonTrackedTarget rightBest = rightResult.getBestTarget();

                if (leftBest.getArea() > rightBest.getArea()) {
                    m_bestTarget = leftBest;
                    m_lastSeenCameraIndex = 0; // Left camera
                } else {
                    m_bestTarget = rightBest;
                    m_lastSeenCameraIndex = 1; // Right camera
                }
                m_hasTarget = true;
            } else if (leftResult.hasTargets()) {
                m_bestTarget = leftResult.getBestTarget();
                m_lastSeenCameraIndex = 0; // Left camera
                m_hasTarget = true;
            } else if (rightResult.hasTargets()) {
                m_bestTarget = rightResult.getBestTarget();
                m_lastSeenCameraIndex = 1; // Right camera
                m_hasTarget = true;
            } else {
                m_hasTarget = false;
                m_bestTarget = null;
                // Keep the last seen camera index for history
            }
        } catch (Exception e) {
            // Handle exceptions from camera operations
            m_hasTarget = false;
            SmartDashboard.putString("Vision/Error", e.getMessage());
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

            // Apply camera offset correction based on which camera is active
            if (m_lastSeenCameraIndex == 0) {
                yaw += VisionConstants.kLeftCameraYawOffset;
            } else if (m_lastSeenCameraIndex == 1) {
                yaw += VisionConstants.kRightCameraYawOffset;
            }

            // Calculate control outputs
            double xSpeed = -m_xController.calculate(pitch, 0);
            double ySpeed = -m_yController.calculate(yaw, 0);
            double rotSpeed = -m_rotController.calculate(m_bestTarget.getSkew(), 0);

            // Clamp speeds to prevent too aggressive movement
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), DriveConstants.kMaxSpeedMetersPerSecond), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), DriveConstants.kMaxSpeedMetersPerSecond), ySpeed);
            rotSpeed = Math.copySign(Math.min(Math.abs(rotSpeed), DriveConstants.kMaxAngularSpeed), rotSpeed);

            // Drive with calculated speeds
            m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
        } else {
            // No target or vision alignment not requested, use manual controls
            double xSpeed = manualX * DriveConstants.kMaxSpeedMetersPerSecond;
            double ySpeed = manualY * DriveConstants.kMaxSpeedMetersPerSecond;
            double rotSpeed = manualRot * DriveConstants.kMaxAngularSpeed;

            m_driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
        }
    }
}