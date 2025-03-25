package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.photonvision.PhotonUtils;

public class AlignToCenterOfTagCommand extends Command {
    private final DriveSubsystem drivetrain;
    private final VisionSubsystem vision;

    // PID constants
    private final double VISION_TURN_kP = 0.1;
    private final double VISION_STRAFE_kP = 0.1;
    private final double TARGET_DISTANCE_METERS = 1.0; // Desired distance from tag

    public AlignToCenterOfTagCommand(DriveSubsystem drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        addRequirements(drivetrain, vision);
    }

    @Override
    public void execute() {
        var target = vision.getFrontCameraTarget(); // Use back camera for center alignment

        if (target != null) {
            // Calculate target yaw - no offset for center alignment
            double targetYaw = target.getYaw();

            // Calculate distance to target
            double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                    0.5, // Camera height
                    1.435, // Target height
                    Math.toRadians(-30.0), // Camera pitch
                    Math.toRadians(target.getPitch()));

            // Calculate drive commands
            double turn = targetYaw * VISION_TURN_kP * Constants.DriveConstants.kMaxAngularSpeed;
            double forward = (TARGET_DISTANCE_METERS - targetRange) * VISION_STRAFE_kP
                    * Constants.DriveConstants.kMaxSpeedMetersPerSecond;

            // Drive
            drivetrain.drive(forward, 0, turn, true);
        } else {
            drivetrain.drive(0, 0, 0, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }
}