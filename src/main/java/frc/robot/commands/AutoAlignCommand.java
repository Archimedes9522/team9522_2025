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

    // PID constants - these will need tuning
    private static final double ROTATION_P = 0.05;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.005;

    private static final double STRAFE_P = 0.1;
    private static final double STRAFE_I = 0.0;
    private static final double STRAFE_D = 0.0;

    public AutoAlignCommand(DriveSubsystem drive, VisionSubsystem vision) {
        this.driveSubsystem = drive;
        this.visionSubsystem = vision;

        rotationController = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
        rotationController.setTolerance(2.0); // 2 degrees of tolerance

        strafeController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);
        strafeController.setTolerance(0.05); // 5cm tolerance

        addRequirements(drive, vision);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTargets()) {
            var target = visionSubsystem.getBestTarget();

            // Calculate rotation correction (yaw)
            double rotationSpeed = -rotationController.calculate(target.getYaw(), 0);

            // Drive to align with target
            driveSubsystem.drive(
                    0, // No forward/backward movement while aligning
                    strafeController.calculate(target.getYaw(), 0), // Strafe to center
                    rotationSpeed, // Rotate to face target
                    true // Field relative
            );
        } else {
            // No target visible, stop moving
            driveSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTargets()) {
            return false;
        }
        // Command is done when we're aligned within tolerance
        return rotationController.atSetpoint() && strafeController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}