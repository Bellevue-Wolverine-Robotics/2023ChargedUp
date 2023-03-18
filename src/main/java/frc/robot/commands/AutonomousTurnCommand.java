package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utils.SmartDashboardUtils;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
    


    public AutonomousTurnCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
    {

        degreesToTurn *= 0.9;//ADJUSTED FOR ROTATIONAL MOMENTUM

        m_driveSubsystem = driveSubsystem;
        m_targetAngle = degreesToTurn;

        m_pid.setTolerance(5);
        m_pid.enableContinuousInput(-180, 180);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
        SmartDashboardUtils.TunablePID(this.getName(), m_pid, DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);

        double motorSpeed = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getGyroDegrees(), m_targetAngle), -0.8, 0.8);
        
        System.out.println("Motor Speed " + motorSpeed);
        System.out.println("CURRENT ANGLE:::::" + m_driveSubsystem.getGyroDegrees());
    
        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

    @Override
    public boolean isFinished()
    {
       // System.out.println("UNDO RETURN FALSE IN AUTONOMOUSTURNCOMMAND");
        //return false;
         return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
        m_driveSubsystem.stopDriveTrain();
    }
}
