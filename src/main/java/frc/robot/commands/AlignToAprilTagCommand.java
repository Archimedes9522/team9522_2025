package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonDriveSubsystem;
import java.util.function.DoubleSupplier;

public class AlignToAprilTagCommand extends Command {
    private final PhotonDriveSubsystem m_photonDrive;
    private final DoubleSupplier m_xSpeedSupplier;
    private final DoubleSupplier m_ySpeedSupplier;
    private final DoubleSupplier m_rotSupplier;

    /**
     * Creates a command that aligns the robot to an AprilTag while still allowing
     * manual control.
     * 
     * @param photonDrive    The photon drive subsystem
     * @param xSpeedSupplier Manual forward/back control
     * @param ySpeedSupplier Manual left/right control
     * @param rotSupplier    Manual rotation control
     */
    public AlignToAprilTagCommand(
            PhotonDriveSubsystem photonDrive,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotSupplier) {

        this.m_photonDrive = photonDrive;
        this.m_xSpeedSupplier = xSpeedSupplier;
        this.m_ySpeedSupplier = ySpeedSupplier;
        this.m_rotSupplier = rotSupplier;

        addRequirements(photonDrive);
    }

    @Override
    public void execute() {
        // Pass manual control values, but enable vision alignment
        m_photonDrive.driveWithVisionAlignment(
                m_xSpeedSupplier.getAsDouble(),
                m_ySpeedSupplier.getAsDouble(),
                m_rotSupplier.getAsDouble(),
                true);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when command ends
        m_photonDrive.driveWithVisionAlignment(0, 0, 0, false);
    }
}