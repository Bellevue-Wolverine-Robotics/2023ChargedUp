package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeGrabCommand extends CommandBase {
    IntakeSubsystem m_intakeSubsystem;

    public IntakeGrabCommand(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        // don't know yet which one to call
        m_intakeSubsystem.intakeForward();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
