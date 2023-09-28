package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceChargeStationCommand extends CommandBase {

    private DriveSubsystem m_driveSubsystem;
    private PIDController m_pid = new PIDController(PIDConstants.kPChargeStation, PIDConstants.kIChargeStation, PIDConstants.kDChargeStation);

    public BalanceChargeStationCommand(DriveSubsystem driveSubsystem) {
        this.m_driveSubsystem = driveSubsystem;

        m_pid.setTolerance(5);
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double pitchDegrees = m_driveSubsystem.getPitchDegrees();
        double pitchRadians = Units.degreesToRadians(pitchDegrees);

        double speed = m_pid.calculate(-Math.sin(pitchRadians), 0);

        if (pitchDegrees < 0) speed *= 1.4;
        
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
