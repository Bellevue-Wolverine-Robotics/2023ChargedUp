package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToggleCommand extends CommandBase {
    IntakeSubsystem m_intakeSubsystem;
    
    public IntakeToggleCommand(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute()
    {
        m_intakeSubsystem.toggleIntake();
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
