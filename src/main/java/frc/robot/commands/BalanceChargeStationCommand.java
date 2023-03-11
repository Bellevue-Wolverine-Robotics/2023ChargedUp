package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

import com.revrobotics.CANSparkMax.IdleMode;

public class BalanceChargeStationCommand extends CommandBase {
    //THIS COMMAND MUST GET RUN EXACTLY WHEN THE FRONT WHEEL STARTS TO TOUCH THE BEGGING OF THE RAMP

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pid = new PIDController(0.05, 0, 0);
    private boolean PIDMode;

    // private double m_thetakP = 0.9;

    public BalanceChargeStationCommand(DriveSubsystem driveSubsystem, boolean PIDMode) {
        this.m_driveSubsystem = driveSubsystem;
        this.PIDMode = PIDMode;

        m_pid.setTolerance(5);
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double pitchDegrees = m_driveSubsystem.getPitchDegreesYWeightedZ();
        double pitchDegreesAbs = Math.abs(pitchDegrees);
        int pitchSign = (int) Math.signum(pitchDegrees);

        // double speed = -MathUtil.clamp(m_pid.calculate(pitchDegrees, 0), -0.5, 0.5);
        // SmartDashboard.putNumber("Balance Speed", speed);
        // System.out.println("Balance Speed: " + speed);

        // if (!m_pid.atSetpoint())
        // {
        //     m_driveSubsystem.tankDrive(speed, speed);
        // }


    
        double speed = this.PIDMode ? MathUtil.clamp(m_pid.calculate(pitchDegrees, 0), -0.5, 0.5): 
                                    Constants.DriveConstants.BALANCE_CHARGESTATION_HARDCODE_TUNE*pitchSign*Math.pow(pitchDegreesAbs, 2);        
        SmartDashboard.putNumber("Balance Speed", speed);
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
