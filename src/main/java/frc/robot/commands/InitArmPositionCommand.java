package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class InitArmPositionCommand extends CommandBase {
    private ArmSubsystem m_armSubsystem;

    public InitArmPositionCommand(ArmSubsystem armSubsystem)
    {
        m_armSubsystem = armSubsystem;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize()
    {
        // slow speed
    }

    @Override
    public void execute()
    {
        m_armSubsystem.rotateArm(0.3);
        if (m_armSubsystem.isSwitchClosed()) m_armSubsystem.resetArmEncoder();

        // System.out.println("initarmposition execute");
    }

    @Override
    public boolean isFinished()
    {
        return m_armSubsystem.isSwitchClosed();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_armSubsystem.stopArmMotor();
    }
}
