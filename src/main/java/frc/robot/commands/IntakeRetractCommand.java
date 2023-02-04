package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    
    public IntakeRetractCommand(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
