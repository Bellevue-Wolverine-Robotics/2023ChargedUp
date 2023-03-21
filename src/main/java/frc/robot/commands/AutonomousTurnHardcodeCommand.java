package frc.robot.commands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousTurnHardcodeCommand extends CommandBase{
    private DriveSubsystem m_driveSubsystem;
    private long _commandStart;
    private double _runTime;
    private double _runSpeed;


    public AutonomousTurnHardcodeCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
    {
        
        
        m_driveSubsystem = driveSubsystem;
        
    
        _runSpeed = degreesToTurn > 0? 0.8: -0.8;
        _commandStart = System.currentTimeMillis();
        _runTime = 1000 * degreesToTurn/50;//guessed hard code time: TEST LATER!!!
        
        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {

        //System.out.println("Motor Speed " + _runSpeed + " Run TIme : " + _runTime);
    
        m_driveSubsystem.tankDrive(_runSpeed, -_runSpeed);
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
