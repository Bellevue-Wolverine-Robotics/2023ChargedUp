package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private PIDController m_pid = new PIDController(0.05, 0, 0.005);
    
    public AutonomousTurnCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
    {
        m_driveSubsystem = driveSubsystem;
        m_targetAngle = degreesToTurn;

        m_pid.setTolerance(1);
        m_pid.enableContinuousInput(-180, 180);


        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
        double motorSpeed = m_pid.calculate(m_driveSubsystem.getGyroDegrees(), m_targetAngle);

        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

    @Override
    public boolean isFinished()
    {
        // return m_pid.atSetpoint();

        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
    }
}
