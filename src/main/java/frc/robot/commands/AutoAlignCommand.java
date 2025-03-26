package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAlignCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final PIDController rotationController;

    // Tune these PID constants
    private static final double ROTATION_P = 0.01; // Start small
    private static final double ROTATION_I = 0.005;
    private static final double ROTATION_D = 0.005;
    private static final double TOLERANCE_DEGREES = 2.0;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;

        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotationController.setTolerance(TOLERANCE_DEGREES);
        rotationController.enableContinuousInput(-180, 180);
        addRequirements(drive, vision);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTargets()) {
            var target = visionSubsystem.getBestTarget();

            // Use yaw directly - this represents degrees off center
            double rotationSpeed = rotationController.calculate(target.getYaw(), 0);

            // Apply a maximum rotation speed limit
            rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);

            // Drive with only rotation
            driveSubsystem.drive(
                    0, // No forward/backward
                    0, // No strafe
                    rotationSpeed,
                    true // Field relative
            );
        } else {
            driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.hasTargets() && rotationController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}