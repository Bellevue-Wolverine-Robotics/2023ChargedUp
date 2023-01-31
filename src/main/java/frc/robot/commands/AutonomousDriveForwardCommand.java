package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousDriveForwardCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private Pose2d m_initialPose;

    private double DISTANCE_METERS = 1.8288;

    public AutonomousDriveForwardCommand (DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_initialPose = m_driveSubsystem.getPose();

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute() {
        System.out.println("Drive forward");

        // todo: FIX ALL THE INVERTED STUFF
        m_driveSubsystem.tankDrive(-0.3,0.3);
        System.out.println("Initial Pose: " + m_initialPose);
        System.out.println("Current pose: " + m_driveSubsystem.getPose());
    }

    @Override
    public boolean isFinished() {
        double poseX = m_driveSubsystem.getPose().getX();
        double poseY = m_driveSubsystem.getPose().getX();

        return Math.sqrt((Math.pow(poseX - m_initialPose.getX(), 2)) + Math.pow(poseY - m_initialPose.getY(), 2)) > DISTANCE_METERS;
    }
}
