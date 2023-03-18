package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDrivestationAbsoluteDegreesCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
    
    public RotateDrivestationAbsoluteDegreesCommand(DriveSubsystem driveSubsystem, double absoluteDegrees)
    {
        // this absolutedegrees is untested.  DO NOT RELY ON COMMAND DURING COMP
        absoluteDegrees *= 0.9;//ADJUSTED FOR ROTATIONAL MOMENTUM

        m_driveSubsystem = driveSubsystem;
        m_targetAngle = absoluteDegrees;

        m_pid.setTolerance(5);
        m_pid.enableContinuousInput(-180, 180);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
        double motorSpeed = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getYaw(), m_targetAngle), -0.8, 0.8);

        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.stopDriveTrain();
    }
}
