package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTestCommand extends CommandBase {
    DriveSubsystem m_driveSubsystem;

    public AutonomousTestCommand (DriveSubsystem m_driveSubsystem) {
        this.m_driveSubsystem = m_driveSubsystem;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.testDrive();
    }

    @Override
    public boolean isFinished() {
        return m_driveSubsystem.getPose().getX() > 1.8288;
    }
}
