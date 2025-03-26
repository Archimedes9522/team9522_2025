package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision"); // Make sure this matches your camera's name
        // Camera mounted on front of robot, half a meter forward of center, half meter
        // up
        robotToCamera = new Transform3d(
                new Translation3d(0.5, 0, 0.5),
                new Rotation3d(0, Math.toRadians(-30), 0) // Angled 30 degrees down
        );
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
        var target = getBestTarget();
        if (target != null) {
            SmartDashboard.putNumber("Target Yaw", target.getYaw());
            SmartDashboard.putNumber("Target ID", target.getFiducialId());
        }
        SmartDashboard.putBoolean("Has Target", hasTargets());
    }
}