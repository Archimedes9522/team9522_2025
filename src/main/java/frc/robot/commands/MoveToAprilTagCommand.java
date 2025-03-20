package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;

public class MoveToAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    public MoveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d targetPose = visionSubsystem.getTargetPose();
        if (targetPose != null) {
            driveSubsystem.resetOdometry(targetPose);
        }
    }

    @Override
    public boolean isFinished() {
        return true; // This command runs once
    }
}