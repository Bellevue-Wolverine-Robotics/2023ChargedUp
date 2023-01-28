package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabRotateCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private CommandJoystick m_joystick;

    public GrabRotateCommand(IntakeSubsystem intakeSubsystem, CommandJoystick joystick)
    {
        m_intakeSubsystem = intakeSubsystem;
        m_joystick = joystick;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute()
    {
        m_intakeSubsystem.rotateGrab(m_joystick.getY());        
    }

}
