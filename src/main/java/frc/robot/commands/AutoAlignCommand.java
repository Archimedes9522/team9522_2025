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

    // PID constants for rotation only
    private static final double ROTATION_P = 0.005;
    private static final double ROTATION_I = 0.0;
    private static final double ROTATION_D = 0.0;
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
            var robotPose = driveSubsystem.getPose();
            var targetPoseOpt = visionSubsystem.getTargetPose(target);

            if (targetPoseOpt.isPresent()) {
                var targetPose = targetPoseOpt.get();

                // Calculate relative vector
                double dx = targetPose.getX() - robotPose.getX();
                double dy = targetPose.getY() - robotPose.getY();

                // Calculate angle relative to current robot heading
                double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
                double currentAngle = robotPose.getRotation().getDegrees();
                double relativeAngle = targetAngle - currentAngle;

                // Normalize to -180 to 180
                relativeAngle = relativeAngle % 360;
                if (relativeAngle > 180)
                    relativeAngle -= 360;
                if (relativeAngle < -180)
                    relativeAngle += 360;

                // Calculate rotation speed
                double rotationSpeed = rotationController.calculate(0, relativeAngle);
                rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);

                // Drive with rotation only
                driveSubsystem.drive(0, 0, rotationSpeed, false);

                System.out.println("RelAngle: " + relativeAngle + " Rot: " + rotationSpeed);
            }
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