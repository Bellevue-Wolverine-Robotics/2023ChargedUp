package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

import com.revrobotics.CANSparkMax.IdleMode;

public class BalanceChargeStationCommand extends CommandBase {
    //THIS COMMAND MUST GET RUN EXACTLY WHEN THE FRONT WHEEL STARTS TO TOUCH THE BEGGING OF THE RAMP

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pid = new PIDController(0.05, 0, 0);
    // private double m_thetakP = 0.9;

    private double distanceFromStart = FieldConstants.RAMP_LENGTH_METERS + FieldConstants.WHOLE_PLATFORM_LENGTH_METER/2;
    private double offSetFactor = 1; //adjust when testing with actrual ramp to account for slip

    public BalanceChargeStationCommand(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        m_driveSubsystem.setMode(IdleMode.kCoast);

        m_pid.setTolerance(2);
    }

    private void setSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        // System.out.println("Motor Speed at: " + speed);
        
        m_driveSubsystem.tankDrive(speed, speed);
    }

    // private bool touchedChargeStation = false;

    @Override
    public void execute() {
        double pitchDegrees = m_driveSubsystem.getPitchDegreesY();
        double pitchRadians = Math.toRadians(pitchDegrees);

        
        SmartDashboard.putNumber("Pitch", pitchRadians);
        // System.out.println("Pitch: " + pitch);
        // System.out.println("Pitch_RADIANS:  " + pitchRadians);
        System.out.println("PITCH_DEGREES: " + pitchDegrees);
        double speed = -MathUtil.clamp(m_pid.calculate(pitchDegrees, 0), -0.5, 0.5);
        
        // P = Fv; P \propto sin(theta) * v
        // double speed = MathUtil.clamp(m_thetakP * Math.sin(pitchRadians), -0.5, 0.5);

        System.out.println("Speed: " + speed);
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
