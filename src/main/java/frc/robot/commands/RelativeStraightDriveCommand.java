package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RelativeStraightDriveCommand extends CommandBase {
    private PIDController m_pidLinear = new PIDController(DriveConstants.kPStraight, DriveConstants.kIStraight, DriveConstants.kDStraight);
    private double m_rotationKP = DriveConstants.kPTurn;

    private DriveSubsystem m_driveSubsystem;

    private double m_targetDistance;
    private double m_targetAngle;

    public RelativeStraightDriveCommand(DriveSubsystem driveSubsystem, double distance){
        m_driveSubsystem = driveSubsystem;
        m_targetDistance = distance; 

        m_pidLinear.setTolerance(0.1);

        m_targetAngle = m_driveSubsystem.getGyroDegrees();

        addRequirements(m_driveSubsystem);
    }


    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getX(), m_targetDistance), -0.5, 0.5);
        double rotationalSpeed = MathUtil.clamp((m_targetAngle - m_driveSubsystem.getGyroDegrees()) * m_rotationKP, -0.5, 0.5);

        // System.out.println("Straight Drive at: " + linearSpeed + " with target at: " + m_targetDistance);
        
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
        m_driveSubsystem.stopDriveTrain();
    }
}
