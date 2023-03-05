package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

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
        m_armSubsystem.rotateArm(m_yInputSupplier.getAsDouble());        
    }

    @Override
    public void end(boolean interrupted)
    {
        m_armSubsystem.stopArmMotor();
    }

}
