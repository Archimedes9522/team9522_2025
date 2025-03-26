package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.DriveSubsystem;
import java.util.Optional;

public class AlignToClosestTagCommand extends Command {
    private final DriveSubsystem m_drive;
    private final PhotonCamera frontCamera;
    private final PhotonCamera backCamera;
    private Command pathCommand;
    private final double TARGET_DISTANCE = 1.0;
    private final AprilTagFieldLayout fieldLayout;

    public AlignToClosestTagCommand(DriveSubsystem driveSubsystem, PhotonCamera frontCam, PhotonCamera backCam) {
        m_drive = driveSubsystem;
        frontCamera = frontCam;
        backCamera = backCam;
        addRequirements(m_drive);

        try {
            fieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    @Override
    public void initialize() {
        pathCommand = null;
        PhotonTrackedTarget closestTarget = findClosestTarget();
        if (closestTarget != null) {
            generatePathToTarget(closestTarget);
        }
    }

    @Override
    public void execute() {
        if (pathCommand == null) {
            end(true);
            return;
        }
    }

    private PhotonTrackedTarget findClosestTarget() {
        var frontResult = frontCamera.getLatestResult();
        var backResult = backCamera.getLatestResult();

        PhotonTrackedTarget closestTarget = null;
        double closestDistance = Double.MAX_VALUE;

        if (frontResult.hasTargets()) {
            for (var target : frontResult.getTargets()) {
                double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                if (dist < closestDistance) {
                    closestDistance = dist;
                    closestTarget = target;
                }
            }
        }

        if (backResult.hasTargets()) {
            for (var target : backResult.getTargets()) {
                double dist = target.getBestCameraToTarget().getTranslation().getNorm();
                if (dist < closestDistance) {
                    closestDistance = dist;
                    closestTarget = target;
                }
            }
        }

        return closestTarget;
    }

    private void generatePathToTarget(PhotonTrackedTarget target) {
        Optional<Pose3d> targetPose = fieldLayout.getTagPose(target.getFiducialId());

        if (targetPose.isPresent()) {
            Pose2d targetPose2d = targetPose.get().toPose2d();
            Pose2d adjustedPose = new Pose2d(
                    targetPose2d.getTranslation().plus(
                            new Translation2d(-TARGET_DISTANCE, 0.0).rotateBy(targetPose2d.getRotation())),
                    targetPose2d.getRotation());

            PathConstraints constraints = new PathConstraints(
                    2.0, // max velocity
                    2.0, // max acceleration
                    360.0, // max angular velocity
                    360.0 // max angular acceleration
            );

            pathCommand = AutoBuilder.pathfindToPose(
                    adjustedPose,
                    constraints,
                    0.0);
            pathCommand.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null && interrupted) {
            pathCommand.cancel();
        }
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished();
    }
}