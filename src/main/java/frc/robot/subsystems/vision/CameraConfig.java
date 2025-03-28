// Camera configuration class to store camera-specific information
package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.VisionSubsystem.TargetObservation;

public class CameraConfig {
    private final PhotonCamera camera;
    private final Transform3d robotToCamera;
    private final String cameraName;
    private final int cameraIndex; // Index for looking up std dev factors
    private boolean connected = false;
    private TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(),
            0.0);
    private Pose3d closestTagPose = new Pose3d();
    private double closestTagDistance = Double.MAX_VALUE;
    private int closestTagId = -1;

    public CameraConfig(String cameraName, Transform3d robotToCamera, int cameraIndex) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraName = cameraName;
        this.robotToCamera = robotToCamera;
        this.cameraIndex = cameraIndex;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    public String getCameraName() {
        return cameraName;
    }

    public int getCameraIndex() {
        return cameraIndex;
    }

    public boolean isConnected() {
        return connected;
    }

    public void setConnected(boolean connected) {
        this.connected = connected;
    }

    public TargetObservation getLatestTargetObservation() {
        return latestTargetObservation;
    }

    public void setLatestTargetObservation(TargetObservation observation) {
        this.latestTargetObservation = observation;
    }

    public Pose3d getClosestTagPose() {
        return closestTagPose;
    }

    public void setClosestTagPose(Pose3d pose, double distance, int tagId) {
        this.closestTagPose = pose;
        this.closestTagDistance = distance;
        this.closestTagId = tagId;
    }

    public void resetClosestTag() {
        this.closestTagPose = new Pose3d();
        this.closestTagDistance = Double.MAX_VALUE;
        this.closestTagId = -1;
    }

    public double getClosestTagDistance() {
        return closestTagDistance;
    }

    public int getClosestTagId() {
        return closestTagId;
    }
}