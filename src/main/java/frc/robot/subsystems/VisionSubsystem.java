package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final PhotonPoseEstimator photonPoseEstimator;

    public VisionSubsystem() {
        camera = new PhotonCamera("frontCam");
        robotToCamera = new Transform3d(
                new Translation3d(0.5, 0, 0.5),
                new Rotation3d(0, Math.toRadians(30), 0));

        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            photonPoseEstimator = new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCamera);
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    public Optional<Pose2d> getTargetPose(PhotonTrackedTarget target) {
        return aprilTagFieldLayout.getTagPose(target.getFiducialId())
                .map(pose3d -> pose3d.toPose2d());
    }

    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        var result = camera.getLatestResult();
        return result.hasTargets() ? result.getBestTarget() : null;
    }

    @Override
    public void periodic() {
        // Update dashboard with vision data
        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            var target = result.getBestTarget();
            SmartDashboard.putNumber("Target Distance", getTargetDistance(target));
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            SmartDashboard.putNumber("Target ID", target.getFiducialId());
        }
        SmartDashboard.putBoolean("Has Target", result.hasTargets());
    }

    public double getTargetDistance(PhotonTrackedTarget target) {
        Transform3d camToTarget = target.getBestCameraToTarget();
        return Math.sqrt(
                camToTarget.getX() * camToTarget.getX() +
                        camToTarget.getY() * camToTarget.getY());
    }
}