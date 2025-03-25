package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;
    private final PhotonCamera frontCamera;

    // Robot-to-camera transforms
    private final Transform2d kRobotToLeftCam;
    private final Transform2d kRobotToRightCam;
    private final Transform2d kRobotToFrontCam;

    private final Field2d debugField = new Field2d();

    // For pose estimation
    private PhotonPoseEstimator frontCameraPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;

    // For simulation
    private VisionSystemSim visionSim;
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;
    private PhotonCameraSim frontCameraSim;

    // Constants for vision alignment
    private final double VISION_TURN_kP = 0.1;
    private final double VISION_STRAFE_kP = 0.1;
    private final double TARGET_DISTANCE_METERS = 1.0; // Default target distance

    public VisionSubsystem() {
        leftCamera = new PhotonCamera("leftCam");
        rightCamera = new PhotonCamera("rightCam");
        frontCamera = new PhotonCamera("frontCam");

        // Define transforms from robot center to each camera
        // These values need to be measured from your robot's center
        kRobotToLeftCam = new Transform2d(new Translation2d(-0.25, 0.25), new Rotation2d(Math.toRadians(-135)));
        kRobotToRightCam = new Transform2d(new Translation2d(-0.25, -0.25), new Rotation2d(Math.toRadians(135)));
        kRobotToFrontCam = new Transform2d(new Translation2d(0.25, 0), new Rotation2d(Math.toRadians(0)));

        SmartDashboard.putData("Vision Field", debugField);

        try {
            aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
            if (aprilTagFieldLayout != null) {
                frontCameraPoseEstimator = new PhotonPoseEstimator(
                        aprilTagFieldLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        frontCamera,
                        kRobotToFrontCam);
                System.out.println("PhotonVision pose estimator initialized successfully");
            } else {
                System.err.println("Failed to load AprilTag field layout");
            }
        } catch (Exception e) {
            System.err.println("Error initializing pose estimator: " + e.getMessage());
            e.printStackTrace();
        }
    }

    // Methods to get target data from each camera
    public PhotonTrackedTarget getLeftCameraTarget() {
        return getNearestTargetFromCamera(leftCamera);
    }

    public PhotonTrackedTarget getRightCameraTarget() {
        return getNearestTargetFromCamera(rightCamera);
    }

    public PhotonTrackedTarget getFrontCameraTarget() {
        return getNearestTargetFromCamera(frontCamera);
    }

    // Find a target with specific AprilTag ID
    public PhotonTrackedTarget getTargetWithID(PhotonCamera camera, int fiducialId) {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            for (var target : results.getTargets()) {
                if (target.getFiducialId() == fiducialId) {
                    return target;
                }
            }
        }
        return null;
    }

    // Calculate distance to target using PhotonUtils
    public double calculateDistanceToTarget(PhotonTrackedTarget target) {
        if (target != null) {
            return PhotonUtils.calculateDistanceToTargetMeters(
                    0.5, // Camera height in meters
                    1.435, // Target height in meters
                    Math.toRadians(-30.0), // Camera pitch in radians
                    Math.toRadians(target.getPitch()));
        }
        return 0.0;
    }

    // Get estimated pose from vision for odometry
    public Optional<Pose2d> getEstimatedGlobalPose() {
        if (frontCameraPoseEstimator == null)
            return Optional.empty();

        var result = frontCameraPoseEstimator.update();
        if (result.isPresent()) {
            var estimatedPose = result.get();
            debugField.getObject("VisionEstimatedPose").setPose(estimatedPose.estimatedPose.toPose2d());
            return Optional.of(estimatedPose.estimatedPose.toPose2d());
        } else {
            return Optional.empty();
        }
    }

    // Get standard deviations for vision measurements
    public double[] getEstimationStdDevs() {
        // Higher standard deviations = less trust in the measurement
        // Lower standard deviations = more trust in the measurement
        return new double[] { 0.5, 0.5, 0.5 }; // x, y, theta in meters and radians
    }

    private PhotonTrackedTarget getNearestTargetFromCamera(PhotonCamera camera) {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            // Find the closest target by area (larger area = closer target)
            return results.getBestTarget();
        }
        return null;
    }

    // For simulation testing
    public void simulationPeriodic(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);

            // Update debug field visualization
            debugField.getObject("SimRobot").setPose(robotPose);
        }
    }

    // Initialize simulation (call this during robot initialization in simulation
    // mode)
    public void initSim() {
        visionSim = new VisionSystemSim("main");

        // Configure camera properties for simulation
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(15);
        cameraProp.setAvgLatencyMs(50);

        // Create camera simulations
        leftCameraSim = new PhotonCameraSim(leftCamera, cameraProp);
        rightCameraSim = new PhotonCameraSim(rightCamera, cameraProp);
        frontCameraSim = new PhotonCameraSim(frontCamera, cameraProp);

        // Add cameras to vision simulation system
        visionSim.addCamera(leftCameraSim, kRobotToLeftCam);
        visionSim.addCamera(rightCameraSim, kRobotToRightCam);
        visionSim.addCamera(frontCameraSim, kRobotToFrontCam);

        // Enable debug wireframes
        leftCameraSim.enableDrawWireframe(true);
        rightCameraSim.enableDrawWireframe(true);
        frontCameraSim.enableDrawWireframe(true);
    }

    // Access the debug field for external use
    public Field2d getSimDebugField() {
        return debugField;
    }

    @Override
    public void periodic() {
        // Display target visibility for each camera
        SmartDashboard.putBoolean("Left Camera Target Visible", getLeftCameraTarget() != null);
        SmartDashboard.putBoolean("Right Camera Target Visible", getRightCameraTarget() != null);
        SmartDashboard.putBoolean("Front Camera Target Visible", getFrontCameraTarget() != null);

        // Update pose estimation if available
        getEstimatedGlobalPose();

        // Display distance to targets if available
        var leftTarget = getLeftCameraTarget();
        var rightTarget = getRightCameraTarget();
        var frontTarget = getFrontCameraTarget();

        if (leftTarget != null) {
            SmartDashboard.putNumber("Left Camera Distance", calculateDistanceToTarget(leftTarget));
        }
        if (rightTarget != null) {
            SmartDashboard.putNumber("Right Camera Distance", calculateDistanceToTarget(rightTarget));
        }
        if (frontTarget != null) {
            SmartDashboard.putNumber("Front Camera Distance", calculateDistanceToTarget(frontTarget));
        }
    }
}