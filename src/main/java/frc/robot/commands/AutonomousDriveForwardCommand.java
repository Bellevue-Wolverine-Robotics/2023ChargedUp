package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDriveForwardCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_distance;

    public AutonomousDriveForwardCommand (DriveSubsystem driveSubsystem, double distance) {
        m_driveSubsystem = driveSubsystem;
        m_distance = distance;

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        System.out.println("Drive forward");

        // todo: FIX ALL THE INVERTED STUFF
        m_driveSubsystem.tankDrive(0.3,0.3);
        System.out.println("Current pose: " + m_driveSubsystem.getPose());
    }

    @Override
    public boolean isFinished() {
        return m_driveSubsystem.getPose().getX() > m_distance;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
    }
}
