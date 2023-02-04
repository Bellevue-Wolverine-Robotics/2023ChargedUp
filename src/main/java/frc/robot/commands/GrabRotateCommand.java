package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Istream.IStream;
import frc.robot.Istream.IStreamBundle;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabRotateCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private IStreamBundle input;

    public GrabRotateCommand(IntakeSubsystem intakeSubsystem, IStreamBundle input)
    {
        this.input = input;

        m_intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute()
    {
        m_intakeSubsystem.rotateArm(input.getY(2));        
    }

}
