package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_initialHeadingDegrees;
    private double m_degreesToTurn = 90;
    
    public AutonomousTurnCommand(DriveSubsystem driveSubsystem)
    {
        m_driveSubsystem = driveSubsystem;
        m_initialHeadingDegrees = driveSubsystem.getRotation2d().getDegrees();

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
        System.out.println("Auto turn command");
        m_driveSubsystem.tankDrive(0.3, -0.3);
    }

    @Override
    public boolean isFinished()
    {
        double currentHeadingDegrees = m_driveSubsystem.getRotation2d().getDegrees();

        return currentHeadingDegrees - m_initialHeadingDegrees > m_degreesToTurn;
    }
}
