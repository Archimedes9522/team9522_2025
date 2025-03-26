package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // PID controllers for alignment
    private final PIDController rotationController;
    private final PIDController strafeController;
    private final PIDController distanceController;

    // Target distance in meters
    private static final double TARGET_DISTANCE = 2.0;
    private static final double ROTATION_TOLERANCE = 2.0; // degrees
    private static final double STRAFE_TOLERANCE = 0.05; // meters
    private static final double DISTANCE_TOLERANCE = 0.1; // meters

    // PID constants - these will need tuning
    private static final double ROTATION_P = 0.05;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.005;

    private static final double STRAFE_P = 0.1;
    private static final double STRAFE_I = 0.0;
    private static final double STRAFE_D = 0.0;

    private static final double DISTANCE_P = 0.1;
    private static final double DISTANCE_I = 0.0;
    private static final double DISTANCE_D = 0.0;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;

        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotationController.setTolerance(2.0); // 2 degrees of tolerance

        strafeController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);
        strafeController.setTolerance(0.05); // 5cm tolerance

        distanceController = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
        distanceController.setTolerance(0.1); // 10cm

        addRequirements(drive, vision);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTargets()) {
            var target = visionSubsystem.getBestTarget();
            var robotPose = driveSubsystem.getPose();
            var targetPoseOpt = visionSubsystem.getTargetPose(target);

            if (targetPoseOpt.isPresent()) {
                var targetPose = targetPoseOpt.get();

                // Calculate vector to target
                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();
                double currentDistance = Math.sqrt(dx * dx + dy * dy);

                // Calculate desired heading to target
                double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

                // Calculate control outputs
                double rotationSpeed = rotationController.calculate(
                        robotPose.getRotation().getDegrees(),
                        targetAngle);

                double strafeSpeed = strafeController.calculate(
                        0, // Current offset
                        target.getYaw() // Desired yaw (center target)
                );

                double forwardSpeed = distanceController.calculate(
                        currentDistance,
                        TARGET_DISTANCE);

                // Drive to align with target
                driveSubsystem.drive(
                        forwardSpeed,
                        strafeSpeed,
                        rotationSpeed,
                        true);
            }
        } else {
            driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.hasTargets() &&
                rotationController.atSetpoint() &&
                strafeController.atSetpoint() &&
                distanceController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}