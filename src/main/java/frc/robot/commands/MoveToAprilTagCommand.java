package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class MoveToAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // PID controllers for movement
    private final PIDController xController = new PIDController(1.0, 0, 0);
    private final PIDController yController = new PIDController(1.0, 0, 0);
    private final PIDController rotController = new PIDController(1.0, 0, 0);

    private Pose2d targetPose;
    private boolean foundTarget = false;

    public MoveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);

        // Configure rotation controller for -pi to pi range
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        // Try to get tag poses from both cameras
        Optional<Pose2d> frontTagPose = visionSubsystem.getLatestFrontTagPose();
        Optional<Pose2d> backTagPose = visionSubsystem.getLatestBackTagPose();

        // Find the closest tag when command starts
        if (frontTagPose.isPresent() && backTagPose.isPresent()) {
            // Both cameras see tags, choose the closest one
            Pose2d robotPose = driveSubsystem.getPose();
            Pose2d front = frontTagPose.get();
            Pose2d back = backTagPose.get();

            double frontDistance = Math.hypot(robotPose.getX() - front.getX(), robotPose.getY() - front.getY());
            double backDistance = Math.hypot(robotPose.getX() - back.getX(), robotPose.getY() - back.getY());

            targetPose = (frontDistance <= backDistance) ? front : back;
            foundTarget = true;
        } else if (frontTagPose.isPresent()) {
            targetPose = frontTagPose.get();
            foundTarget = true;
        } else if (backTagPose.isPresent()) {
            targetPose = backTagPose.get();
            foundTarget = true;
        } else {
            targetPose = null;
            foundTarget = false;
        }

        if (foundTarget) {
            SmartDashboard.putString("Target Pose",
                    String.format("X: %.2f, Y: %.2f, Rot: %.2f",
                            targetPose.getX(), targetPose.getY(),
                            targetPose.getRotation().getDegrees()));
        } else {
            SmartDashboard.putString("Target Pose", "No target found");
        }
    }

    @Override
    public void execute() {
        if (!foundTarget) {
            return; // No target to move to
        }

        // Use the drive subsystem to move toward the target
        double xSpeed = xController.calculate(driveSubsystem.getPose().getX(), targetPose.getX());
        double ySpeed = yController.calculate(driveSubsystem.getPose().getY(), targetPose.getY());
        double rotSpeed = rotController.calculate(
                driveSubsystem.getPose().getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        // Apply speed scaling and drive
        xSpeed = MathUtil.clamp(xSpeed, -0.5, 0.5);
        ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);
        rotSpeed = MathUtil.clamp(rotSpeed, -0.5, 0.5);

        driveSubsystem.drive(xSpeed, ySpeed, rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        if (!foundTarget || targetPose == null) {
            return true; // End if no target was found or targetPose wasn't set
        }

        // Check if we've reached the target
        Pose2d currentPose = driveSubsystem.getPose();
        double distanceToTarget = Math.hypot(
                currentPose.getX() - targetPose.getX(),
                currentPose.getY() - targetPose.getY());

        return distanceToTarget < 0.1; // Within 10cm of target
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive when we're done
        driveSubsystem.drive(0, 0, 0, true);
    }
}