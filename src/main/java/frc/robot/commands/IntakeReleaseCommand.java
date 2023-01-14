package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReleaseCommand extends CommandBase {
    IntakeSubsystem m_intakeSubsystem;
    
    public IntakeReleaseCommand(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute() {
        // don't know yet which one to call
        System.out.println("Release");
        m_intakeSubsystem.retractIntake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
