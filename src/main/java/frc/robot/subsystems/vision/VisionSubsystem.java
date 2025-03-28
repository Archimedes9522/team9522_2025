package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class VisionSubsystem extends SubsystemBase {
    // Camera hardware
    private final List<CameraConfig> cameras = new ArrayList<>();

    // Field layout
    private final AprilTagFieldLayout aprilTagLayout;

    // Vision consumer (optional callback for pose estimates)
    private final VisionConsumer consumer;

    // Current vision state
    private List<PoseObservation> poseObservationsList = new ArrayList<>();
    private PoseObservation[] poseObservations = new PoseObservation[0];
    private Pose3d closestTag = new Pose3d();
    private int[] tagIds = new int[0];

    /**
     * Creates a new VisionSubsystem with default cameras from VisionConstants.
     *
     * @param consumer Optional consumer for vision pose estimates
     */
    public VisionSubsystem(VisionConsumer consumer) {
        // Set up cameras using VisionConstants
        this.cameras.add(new CameraConfig(
                VisionConstants.camera0Name,
                VisionConstants.robotToCamera0,
                0));

        this.cameras.add(new CameraConfig(
                VisionConstants.camera1Name,
                VisionConstants.robotToCamera1,
                1));

        this.aprilTagLayout = VisionConstants.aprilTagLayout;
        this.consumer = consumer;
    }

    /**
     * Creates a new VisionSubsystem with custom cameras.
     *
     * @param cameraConfigs  List of camera configurations (name and transform
     *                       pairs)
     * @param aprilTagLayout April tag field layout
     * @param consumer       Optional consumer for vision pose estimates
     */
    public VisionSubsystem(
            List<Map.Entry<String, Transform3d>> cameraConfigs,
            AprilTagFieldLayout aprilTagLayout,
            VisionConsumer consumer) {

        int cameraIndex = 0;
        for (var config : cameraConfigs) {
            this.cameras.add(new CameraConfig(config.getKey(), config.getValue(), cameraIndex++));
        }

        this.aprilTagLayout = aprilTagLayout;
        this.consumer = consumer;
    }

    @Override
    public void periodic() {
        // Reset observations for this cycle
        poseObservationsList.clear();
        Set<Integer> tagIdSet = new HashSet<>();

        // Reset closest tag for each camera
        for (CameraConfig cameraConfig : cameras) {
            cameraConfig.resetClosestTag();
        }

        // Process each camera
        for (CameraConfig cameraConfig : cameras) {
            PhotonCamera camera = cameraConfig.getCamera();

            // Update connection status
            boolean connected = camera.isConnected();
            cameraConfig.setConnected(connected);

            if (!connected) {
                DriverStation.reportWarning("Camera " + cameraConfig.getCameraName() + " disconnected", false);
                continue;
            }

            // Read new camera observations
            Transform3d robotToCamera = cameraConfig.getRobotToCamera();
            int cameraIndex = cameraConfig.getCameraIndex();

            // Get camera-specific standard deviation factor (with bounds checking)
            double cameraStdDevFactor = 1.0;
            if (cameraIndex >= 0 && cameraIndex < VisionConstants.cameraStdDevFactors.length) {
                cameraStdDevFactor = VisionConstants.cameraStdDevFactors[cameraIndex];
            }

            for (var result : camera.getAllUnreadResults()) {
                // Update latest target observation for this camera
                if (result.hasTargets()) {
                    cameraConfig.setLatestTargetObservation(new TargetObservation(
                            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                            Rotation2d.fromDegrees(result.getBestTarget().getPitch()),
                            PhotonUtils.calculateDistanceToTargetMeters(
                                    robotToCamera.getZ(),
                                    0.5, // Target height - adjust as needed
                                    robotToCamera.getRotation().getY(),
                                    Units.degreesToRadians(result.getBestTarget().getPitch()))));
                } else {
                    cameraConfig
                            .setLatestTargetObservation(new TargetObservation(new Rotation2d(), new Rotation2d(), 0.0));
                }

                // Process multi-tag results
                if (result.multitagResult.isPresent()) {
                    var multitagResult = result.multitagResult.get();

                    // Skip results with too much ambiguity
                    if (multitagResult.estimatedPose.ambiguity > VisionConstants.maxAmbiguity) {
                        continue;
                    }

                    // Calculate robot pose
                    Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Calculate average tag distance
                    double totalTagDistance = 0.0;
                    for (var target : result.targets) {
                        totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                    }

                    // Add tag IDs
                    for (Short id : multitagResult.fiducialIDsUsed) {
                        tagIdSet.add(id.intValue());
                    }

                    // Add observation
                    PoseObservationType poseType = PoseObservationType.PHOTONVISION;
                    poseObservationsList.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(),
                                    robotPose,
                                    multitagResult.estimatedPose.ambiguity,
                                    multitagResult.fiducialIDsUsed.size(),
                                    totalTagDistance / result.targets.size(),
                                    poseType));

                    // Update camera-specific closest tag
                    double avgDistance = totalTagDistance / result.targets.size();
                    if (avgDistance < cameraConfig.getClosestTagDistance()) {
                        int tagId = -1;
                        if (!multitagResult.fiducialIDsUsed.isEmpty()) {
                            tagId = multitagResult.fiducialIDsUsed.get(0);
                        }
                        cameraConfig.setClosestTagPose(robotPose, avgDistance, tagId);
                    }

                    // Send to pose estimator if consumer exists
                    if (consumer != null) {
                        // Calculate standard deviation based on distance and tag count
                        double tagCount = multitagResult.fiducialIDsUsed.size();

                        // Increase standard deviation with distance, decrease with more tags
                        double linearStdDev = VisionConstants.linearStdDevBaseline * avgDistance / Math.sqrt(tagCount);
                        double angularStdDev = VisionConstants.angularStdDevBaseline * avgDistance
                                / Math.sqrt(tagCount);

                        // Apply camera-specific factor
                        linearStdDev *= cameraStdDevFactor;
                        angularStdDev *= cameraStdDevFactor;

                        // Apply MegaTag-specific factors if applicable
                        if (poseType == PoseObservationType.MEGATAG_2) {
                            linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
                            angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
                        }

                        // Create standard deviation vector
                        var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

                        // Add pose to estimator
                        consumer.accept(robotPose.toPose2d(), result.getTimestampSeconds(), stdDevs);
                    }

                } else if (!result.targets.isEmpty()) {
                    // Single tag processing
                    var target = result.getBestTarget();

                    // Skip results with too much ambiguity
                    if (target.getPoseAmbiguity() > VisionConstants.maxAmbiguity) {
                        continue;
                    }

                    // Get tag pose from field layout
                    var tagPose = aprilTagLayout.getTagPose(target.getFiducialId());
                    if (tagPose.isPresent()) {
                        Transform3d cameraToTarget = target.getBestCameraToTarget();

                        // Calculate robot pose
                        Transform3d targetToCamera = cameraToTarget.inverse();
                        Transform3d targetToRobot = targetToCamera.plus(robotToCamera);
                        Pose3d robotPose = tagPose.get().transformBy(targetToRobot);

                        // Add tag ID
                        tagIdSet.add(target.getFiducialId());

                        // Add observation
                        PoseObservationType poseType = PoseObservationType.PHOTONVISION;
                        double tagDistance = cameraToTarget.getTranslation().getNorm();
                        poseObservationsList.add(
                                new PoseObservation(
                                        result.getTimestampSeconds(),
                                        robotPose,
                                        target.getPoseAmbiguity(),
                                        1,
                                        tagDistance,
                                        poseType));

                        // Update camera-specific closest tag
                        if (tagDistance < cameraConfig.getClosestTagDistance()) {
                            cameraConfig.setClosestTagPose(robotPose, tagDistance, target.getFiducialId());
                        }

                        // Send to pose estimator if consumer exists
                        if (consumer != null) {
                            // Calculate standard deviation based on distance and tag count
                            double linearStdDev = VisionConstants.linearStdDevBaseline * tagDistance;
                            double angularStdDev = VisionConstants.angularStdDevBaseline * tagDistance;

                            // Apply camera-specific factor
                            linearStdDev *= cameraStdDevFactor;
                            angularStdDev *= cameraStdDevFactor;

                            // Create standard deviation vector
                            var stdDevs = VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);

                            // Add pose to estimator
                            consumer.accept(robotPose.toPose2d(), result.getTimestampSeconds(), stdDevs);
                        }
                    }
                }
            }
        }

        // Update arrays with collected data from all cameras
        poseObservations = poseObservationsList.toArray(new PoseObservation[0]);
        tagIds = tagIdSet.stream().mapToInt(Integer::intValue).toArray();

        // Update closest tag
        updateClosestTag();
    }

    /**
     * Update the closest tag based on current observations
     */
    private void updateClosestTag() {
        if (poseObservations.length == 0) {
            return;
        }

        // Find the observation with the smallest distance
        double minDistance = Double.MAX_VALUE;
        Pose3d closestPose = null;

        for (var observation : poseObservations) {
            if (observation.averageTagDistance < minDistance) {
                minDistance = observation.averageTagDistance;
                closestPose = observation.pose;
            }
        }

        if (closestPose != null) {
            this.closestTag = closestPose;
        }
    }

    /**
     * Gets the closest tag relative to the current pose.
     * 
     * @param currentPose The current robot pose
     * @return The closest tag pose
     */
    public Pose2d getClosestTagPose(Pose2d currentPose) {
        return getClosestTag().toPose2d();
    }

    /**
     * Gets the closest tag pose observed by the specified camera
     * 
     * @param cameraIndex The index of the camera
     * @param currentPose The current robot pose (used to filter by distance if
     *                    needed)
     * @return The closest tag pose relative to the specified camera
     */
    public Pose2d getClosestTagPoseForCamera(int cameraIndex, Pose2d currentPose) {
        if (cameraIndex < 0 || cameraIndex >= cameras.size()) {
            return new Pose2d();
        }
        return cameras.get(cameraIndex).getClosestTagPose().toPose2d();
    }

    /**
     * @return The closest April tag pose
     */
    public Pose3d getClosestTag() {
        return closestTag;
    }

    /**
     * @return Array of tag IDs seen in the latest update across all cameras
     */
    public int[] getTagIds() {
        return tagIds;
    }

    /**
     * @return Whether any camera in the vision system is connected
     */
    public boolean isConnected() {
        for (CameraConfig camera : cameras) {
            if (camera.isConnected()) {
                return true;
            }
        }
        return false;
    }

    /**
     * @param cameraIndex The index of the camera to check
     * @return Whether the specified camera is connected
     */
    public boolean isConnected(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameras.size()) {
            return false;
        }
        return cameras.get(cameraIndex).isConnected();
    }

    /**
     * @return Array of pose observations from the latest update across all cameras
     */
    public PoseObservation[] getPoseObservations() {
        return poseObservations;
    }

    /**
     * @return The number of cameras in the subsystem
     */
    public int getCameraCount() {
        return cameras.size();
    }

    /**
     * Gets a camera configuration by index
     * 
     * @param cameraIndex The index of the camera
     * @return The camera configuration or null if index is invalid
     */
    public CameraConfig getCamera(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= cameras.size()) {
            return null;
        }
        return cameras.get(cameraIndex);
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty, double tr) {
    }

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {
    }

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}