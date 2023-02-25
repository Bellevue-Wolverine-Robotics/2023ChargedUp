package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Istream.IStream;
import frc.robot.Istream.IStreamBundle;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateArmSpeedCommand extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private DoubleSupplier m_yInputSupplier;

    public RotateArmSpeedCommand(ArmSubsystem m_armSubsystem, DoubleSupplier yInputSupplier)
    {
        m_yInputSupplier = yInputSupplier;

        this.m_armSubsystem = m_armSubsystem;
        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute()
    {
        // lol
        // square inputs, then multiply by sign
        m_armSubsystem.rotateArm(Math.pow(m_yInputSupplier.getAsDouble(), 2) * (int) Math.signum(m_yInputSupplier.getAsDouble()));        
    }

    @Override
    public void end(boolean interrupted)
    {
        m_armSubsystem.stopArmMotor();
    }

}
