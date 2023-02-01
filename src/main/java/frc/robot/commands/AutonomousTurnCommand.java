package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private boolean m_aboveAngle;
    
    public AutonomousTurnCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
    {
        m_driveSubsystem = driveSubsystem;

        m_targetAngle = degreesToTurn;
        m_aboveAngle = driveSubsystem.getGyroDegrees() > m_targetAngle;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
        System.out.println("Auto turn command");

        if (m_aboveAngle)
        {
            m_driveSubsystem.tankDrive(-0.3, 0.3);
        }
        else
        {
            m_driveSubsystem.tankDrive(0.3, -0.3);
        }
    }

    @Override
    public boolean isFinished()
    {
        double currentHeadingDegrees = m_driveSubsystem.getGyroDegrees();

        if (m_aboveAngle)
        {
            return currentHeadingDegrees < m_targetAngle;
        }
        else
        {
            return currentHeadingDegrees > m_targetAngle;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
    }
}
