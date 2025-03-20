package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera frontCamera = new PhotonCamera("FrontCamera");
    private final PhotonCamera backCamera = new PhotonCamera("BackCamera");

    public VisionSubsystem() {
        // Initialize cameras if needed
    }

    public Pose2d getTargetPose() {
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult backResult = backCamera.getLatestResult();

        PhotonTrackedTarget bestTarget = null;
        if (frontResult.hasTargets()) {
            bestTarget = frontResult.getBestTarget();
        } else if (backResult.hasTargets()) {
            bestTarget = backResult.getBestTarget();
        }

        if (bestTarget != null) {
            Transform3d transform = bestTarget.getBestCameraToTarget();
            Translation2d translation = new Translation2d(transform.getX(), transform.getY());
            Rotation2d rotation = new Rotation2d(transform.getRotation().getZ());
            return new Pose2d(translation, rotation);
        }

        return null; // No targets found
    }
}