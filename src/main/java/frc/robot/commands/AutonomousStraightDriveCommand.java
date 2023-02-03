package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousStraightDriveCommand extends CommandBase {
    private PIDController m_pidLinear = new PIDController(1, 0, 0.1);
    private PIDController m_pidRotational = new PIDController(0.02, 0, 0.001);

    private DriveSubsystem m_driveSubsystem;

    private double m_distance;
    private double m_initialAngle;

    public AutonomousStraightDriveCommand(DriveSubsystem driveSubsystem, double distance){
        this.m_driveSubsystem = driveSubsystem;
        this.m_distance = distance; 

        m_pidLinear.setTolerance(0.1);

        m_pidRotational.setTolerance(5);
        m_pidRotational.enableContinuousInput(-180, 180);

        this.m_initialAngle = m_driveSubsystem.getGyroDegrees();

        addRequirements(m_driveSubsystem);
    }


    @Override
    public void execute() {
        double linearSpeed = MathUtil.clamp(m_pidLinear.calculate(m_driveSubsystem.getPose().getX(), m_distance), -0.5, 0.5);
        double rotationSpeed = m_pidRotational.calculate(m_driveSubsystem.getGyroDegrees(), m_initialAngle);

        if (m_pidRotational.atSetpoint()) {
            m_driveSubsystem.tankDrive(linearSpeed , linearSpeed);
        } else {
            m_driveSubsystem.tankDrive(rotationSpeed, -rotationSpeed);
            
        }
    }
}
