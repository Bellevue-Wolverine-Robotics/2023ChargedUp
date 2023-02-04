package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDriveForwardCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_distance;
    private PIDController m_pid = new PIDController(0.1, 0, 0); 


    public AutonomousDriveForwardCommand (DriveSubsystem driveSubsystem, double distance) {
        m_driveSubsystem = driveSubsystem;
        m_distance = distance;

        m_pid.setTolerance(0.1);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() { 
        double motorSpeed = m_pid.calculate(m_driveSubsystem.getPose().getX(), m_distance);
        
        m_driveSubsystem.tankDrive(motorSpeed, motorSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
    }
}
