package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.lang.Math;

public class BalanceChargeStation extends CommandBase {
    //THIS COMMAND MUST GET RUN EXACTLY WHEN THE FRONT WHEEL STARTS TO TOUCH THE BEGGING OF THE RAMP

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pid = new PIDController(0.1, 0, 0);

    private double distanceFromStart = FieldConstants.RAMP_LENGTH_METERS + FieldConstants.WHOLE_PLATFORM_LENGTH_METER/2;
    private double offSetFactor = 1; //adjust when testing with actrual ramp to account for slip

    public BalanceChargeStation(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

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
        double pitch = m_driveSubsystem.getPitchDegreesY(); // Getting the angle of the robot
        // if (this.touchedChargeStation == false) { //
        //     m_driveSubsystem.tankDrive(0.25, 0.25);
        //     if (angle > 10) {
        //         this.touchedChargeStation = true;
        //     }
        // } else {
        //     m_driveSubsystem.tankDrive(angle / 15, angle / 15);
        // }
        
        SmartDashboard.putNumber("Pitch", pitch);
        // System.out.println("Pitch: " + pitch);

        double speed = MathUtil.clamp(m_pid.calculate(pitch, 0), -0.5, 0.5);
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
        m_driveSubsystem.stopDriveTrain();
    }
}
