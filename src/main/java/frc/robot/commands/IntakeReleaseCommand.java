package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReleaseCommand extends CommandBase {
    IntakeSubsystem m_intakeSubsystem;
    
    public IntakeReleaseCommand(IntakeSubsystem intakeSubsystem)
    {
        m_intakeSubsystem = intakeSubsystem;

    }
}
