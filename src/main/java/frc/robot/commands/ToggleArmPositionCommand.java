package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleArmPositionCommand extends Command {
    private final ClimberSubsystem climberSubsystem;

    public ToggleArmPositionCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        climberSubsystem.togglePositionCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}