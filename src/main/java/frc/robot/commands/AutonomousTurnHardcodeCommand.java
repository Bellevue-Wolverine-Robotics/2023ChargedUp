package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utils.SmartDashboardUtils;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnHardcodeCommand extends CommandBase{
    private DriveSubsystem m_driveSubsystem;
    private long _commandStart;
    private double _runTime;


    public AutonomousTurnHardcodeCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
    {
        
        m_driveSubsystem = driveSubsystem;
        
        _commandStart = System.currentTimeMillis();
        _runTime = 1000 * degreesToTurn/10;//guessed hard code time: TEST LATER!!!
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {

        // System.out.println("Motor Speed " + motorSpeed);
        double motorSpeed = 0.5;//half speed during turn;
        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return _commandStart + _runTime < System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.resetPose();
        m_driveSubsystem.stopDriveTrain();
    }
}
