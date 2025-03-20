package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
    private final PhotonCamera backCamera = new PhotonCamera("BackCamera");
    private final PhotonCamera driverCamera = new PhotonCamera("DriverCamera");

    // Camera positions relative to robot center
    private final Transform3d frontCameraToRobot = new Transform3d(); // Fill with actual values
    private final Transform3d backCameraToRobot = new Transform3d(); // Fill with actual values

    // Store the latest detected tag poses
    private Pose2d latestFrontTagPose = null;
    private Pose2d latestBackTagPose = null;

    // Track whether we have valid vision data
    private boolean hasFrontVision = false;
    private boolean hasBackVision = false;

    public VisionSubsystem() {
        // Set driver camera to use a different pipeline if needed
        driverCamera.setDriverMode(true);
    }

    @Override
    public void periodic() {
        // Process data from front camera
        processVisionData(frontCamera, true);

        // Process data from back camera
        processVisionData(backCamera, false);

        // Output data to SmartDashboard for debugging
        SmartDashboard.putBoolean("Front Vision Valid", hasFrontVision);
        SmartDashboard.putBoolean("Back Vision Valid", hasBackVision);
    }

    private void processVisionData(PhotonCamera camera, boolean isFrontCamera) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

            // Calculate robot pose based on target and camera position
            Pose2d targetPose = calculateTargetPose(cameraToTarget,
                    isFrontCamera ? frontCameraToRobot : backCameraToRobot);

            // Update the appropriate pose
            if (isFrontCamera) {
                latestFrontTagPose = targetPose;
                hasFrontVision = true;
            } else {
                latestBackTagPose = targetPose;
                hasBackVision = true;
            }
        } else {
            // No targets found
            if (isFrontCamera) {
                hasFrontVision = false;
            } else {
                hasBackVision = false;
            }
        }
    }

    private Pose2d calculateTargetPose(Transform3d cameraToTarget, Transform3d cameraToRobot) {
        // Implement pose calculation based on transforms
        // This is a placeholder - actual implementation would depend on field setup
        Translation2d translation = new Translation2d(cameraToTarget.getX(), cameraToTarget.getY());
        Rotation2d rotation = new Rotation2d(cameraToTarget.getRotation().getZ());
        return new Pose2d(translation, rotation);
    }

    public Optional<Pose2d> getLatestFrontTagPose() {
        return (hasFrontVision && latestFrontTagPose != null) ? Optional.of(latestFrontTagPose) : Optional.empty();
    }

    public Optional<Pose2d> getLatestBackTagPose() {
        return (hasBackVision && latestBackTagPose != null) ? Optional.of(latestBackTagPose) : Optional.empty();
    }
}