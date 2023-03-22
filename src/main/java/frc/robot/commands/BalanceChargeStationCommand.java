package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

import com.revrobotics.CANSparkMax.IdleMode;

public class BalanceChargeStationCommand extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pid = new PIDController(PIDConstants.kPChargeStation, PIDConstants.kIChargeStation, PIDConstants.kDChargeStation);

    // private double m_thetakP = 0.9;
/*
  public static class PIDConstants {
    public static final double kPChargeStation = 1.5;
    public static final double kIChargeStation = 0;
    public static final double kDChargeStation = 0.015;   
  }
 */
    public BalanceChargeStationCommand(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        m_pid.setTolerance(5);
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double pitchDegrees = m_driveSubsystem.getPitchDegrees();
        double pitchRadians = Units.degreesToRadians(pitchDegrees);

        // double speed = this.PIDMode ? MathUtil.clamp(m_pid.calculate(pitchDegrees, 0), -0.5, 0.5): 
        //                             Constants.DriveConstants.BALANCE_CHARGESTATION_HARDCODE_TUNE*pitchSign*Math.pow(pitchDegreesAbs, 2);        
        double speed = m_pid.calculate(Math.sin(pitchRadians), 0);

        System.out.println("Charge Balance");

        //TO ADJUST FOR SLOWER SEED WHEN REVERSING  
        if(speed < 0){
            speed *= 1.1;
        }
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        m_driveSubsystem.tankDrive(speed, speed);
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
        // m_driveSubsystem.stopDriveTrain();
    }
}
