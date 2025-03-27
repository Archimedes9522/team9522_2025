// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {
    private final VisionConsumer consumer;
    private final VisionIOInputsAutoLogged[] inputs;

    // Camera objects
    private final PhotonCamera[] cameras;
    private final Transform3d[] robotToCameras;
    private final Alert[] disconnectedAlerts;

    // Vision data
    private boolean[] connected;
    private TargetObservation[] latestTargetObservations;
    private List<PoseObservation>[] allPoseObservations;
    private Set<Integer>[] tagIds;

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {
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

    public Pose2d getClosestTagPose(int cameraIndex, Pose2d currentPose) {
        // Check if the closest tag is essentially at origin (0,0,0), indicating no
        // valid detection
        Pose2d tagPose = inputs[cameraIndex].closestTag.toPose2d();

        // Check if this is essentially the origin
        if (Math.abs(tagPose.getX()) < 0.001 && Math.abs(tagPose.getY()) < 0.001) {
            // No valid tag detected, return robot pose instead (no darting off to origin on
            // accident)
            return currentPose;
        }

        return tagPose;
    }

    /**
     * Creates a new VisionSubsystem with specified cameras.
     * 
     * @param consumer       The consumer that receives vision measurements for pose
     *                       estimation
     * @param cameraNames    Array of camera names as configured in PhotonVision
     * @param robotToCameras Array of transforms from robot to each camera
     */
    @SuppressWarnings("unchecked")
    public VisionSubsystem(VisionConsumer consumer, String[] cameraNames, Transform3d[] robotToCameras) {
        this.consumer = consumer;
        this.robotToCameras = robotToCameras;

        // Initialize camera objects
        int cameraCount = cameraNames.length;
        this.cameras = new PhotonCamera[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            cameras[i] = new PhotonCamera(cameraNames[i]);
        }

        // Initialize data structures
        this.connected = new boolean[cameraCount];
        this.latestTargetObservations = new TargetObservation[cameraCount];
        this.allPoseObservations = (List<PoseObservation>[]) new List[cameraCount];
        this.tagIds = (Set<Integer>[]) new Set[cameraCount];

        for (int i = 0; i < cameraCount; i++) {
            latestTargetObservations[i] = new TargetObservation(new Rotation2d(), new Rotation2d());
            allPoseObservations[i] = new LinkedList<>();
            tagIds[i] = new HashSet<>();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[cameraCount];
        for (int i = 0; i < cameraCount; i++) {
            disconnectedAlerts[i] = new Alert(
                    "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Creates a new VisionSubsystem with a single camera.
     * 
     * @param consumer      The consumer that receives vision measurements for pose
     *                      estimation
     * @param cameraName    The name of the camera as configured in PhotonVision
     * @param robotToCamera The transform from robot to camera
     */
    public VisionSubsystem(VisionConsumer consumer, String cameraName, Transform3d robotToCamera) {
        this(consumer, new String[] { cameraName }, new Transform3d[] { robotToCamera });
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return latestTargetObservations[cameraIndex].tx();
    }

    @Override
    public void periodic() {
        // Update camera data
        updateCameraData();

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < cameras.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!connected[cameraIndex]);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : tagIds[cameraIndex]) {
                var tagPose = aprilTagLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop over pose observations
            for (var observation : allPoseObservations[cameraIndex]) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > aprilTagLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > aprilTagLayout.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = linearStdDevBaseline * stdDevFactor;
                double angularStdDev = angularStdDevBaseline * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= linearStdDevMegatag2Factor;
                    angularStdDev *= angularStdDevMegatag2Factor;
                }
                if (cameraIndex < cameraStdDevFactors.length) {
                    linearStdDev *= cameraStdDevFactors[cameraIndex];
                    angularStdDev *= cameraStdDevFactors[cameraIndex];
                }

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera data
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

        // Clear observations for next cycle
        for (List<PoseObservation> observations : allPoseObservations) {
            observations.clear();
        }
        for (Set<Integer> ids : tagIds) {
            ids.clear();
        }
    }

    /**
     * Updates the data from all cameras.
     */
    private void updateCameraData() {
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            Transform3d robotToCamera = robotToCameras[i];

            // Check connection status
            connected[i] = camera.isConnected();

            // Read new camera observations
            for (var result : camera.getAllUnreadResults()) {
                // Update latest target observation
                if (result.hasTargets()) {
                    latestTargetObservations[i] = new TargetObservation(
                            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
                } else {
                    latestTargetObservations[i] = new TargetObservation(new Rotation2d(), new Rotation2d());
                }

                // Add pose observation
                if (result.multitagResult.isPresent()) { // Multitag result
                    var multitagResult = result.multitagResult.get();

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
                    for (int tagId : multitagResult.fiducialIDsUsed) {
                        tagIds[i].add(tagId);
                    }

                    // Add observation
                    allPoseObservations[i].add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    multitagResult.estimatedPose.ambiguity, // Ambiguity
                                    multitagResult.fiducialIDsUsed.size(), // Tag count
                                    totalTagDistance / result.targets.size(), // Average tag distance
                                    PoseObservationType.PHOTONVISION)); // Observation type

                } else if (!result.targets.isEmpty()) { // Single tag result
                    var target = result.targets.get(0);

                    // Calculate robot pose
                    var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                    if (tagPose.isPresent()) {
                        Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(),
                                tagPose.get().getRotation());
                        Transform3d cameraToTarget = target.bestCameraToTarget;
                        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                        // Add tag ID
                        tagIds[i].add(target.fiducialId);

                        // Add observation
                        allPoseObservations[i].add(
                                new PoseObservation(
                                        result.getTimestampSeconds(), // Timestamp
                                        robotPose, // 3D pose estimate
                                        target.poseAmbiguity, // Ambiguity
                                        1, // Tag count
                                        cameraToTarget.getTranslation().getNorm(), // Average tag distance
                                        PoseObservationType.PHOTONVISION)); // Observation type
                    }
                }
            }
        }
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
