package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousStraightDriveCommand extends CommandBase {
    private PIDController m_pidLinear = new PIDController(2, 0, 0.2);
    private double m_rotationKP = 0.02;

    private DriveSubsystem m_driveSubsystem;

    private double m_targetDistance;
    private double m_targetAngle;

    public AutonomousStraightDriveCommand(DriveSubsystem driveSubsystem, double distance){
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = distance; 

        m_pidLinear.setTolerance(0.05);

        m_targetAngle = m_driveSubsystem.getGyroDegrees();

        addRequirements(m_driveSubsystem);
    }


    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getX(), m_targetDistance), -0.5, 0.5);
        double rotationalSpeed = MathUtil.clamp((m_targetAngle - m_driveSubsystem.getGyroDegrees()) * m_rotationKP, -0.5, 0.5);

        m_driveSubsystem.tankDrive(linearSpeed + rotationalSpeed, linearSpeed -rotationalSpeed);
    }

    @Override
    public boolean isFinished() {
        double rotationError = Math.abs(m_driveSubsystem.getGyroDegrees() - m_targetAngle);

        return m_pidLinear.atSetpoint() && rotationError < 0.1;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
    }
}
