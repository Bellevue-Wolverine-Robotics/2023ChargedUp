package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Utils.SmartDashboardUtils;
import frc.robot.subsystems.DriveSubsystem;

public class RotateDrivestationAbsoluteDegreesCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private double m_targetAngle;
    private PIDController m_pid = new PIDController(DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);
    
<<<<<<< HEAD:src/main/java/frc/robot/commands/AutonomousTurnCommand.java


    public AutonomousTurnCommand(DriveSubsystem driveSubsystem, double degreesToTurn)
=======
    public RotateDrivestationAbsoluteDegreesCommand(DriveSubsystem driveSubsystem, double absoluteDegrees)
>>>>>>> 5ac609f6fafa0cd26c7dfeb401baa0ef64498b14:src/main/java/frc/robot/commands/RotateDrivestationAbsoluteDegreesCommand.java
    {

        degreesToTurn *= 0.9;//ADJUSTED FOR ROTATIONAL MOMENTUM

        m_driveSubsystem = driveSubsystem;
        m_targetAngle = absoluteDegrees;

        m_pid.setTolerance(5);
        m_pid.enableContinuousInput(-180, 180);

        addRequirements(m_driveSubsystem);
    }

    @Override
    public void execute()
    {
<<<<<<< HEAD:src/main/java/frc/robot/commands/AutonomousTurnCommand.java
        SmartDashboardUtils.TunablePID(this.getName(), m_pid, DriveConstants.kPTurn, DriveConstants.kITurn, DriveConstants.kDTurn);

        double motorSpeed = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getGyroDegrees(), m_targetAngle), -0.8, 0.8);
        
        System.out.println("Motor Speed " + motorSpeed);
        System.out.println("CURRENT ANGLE:::::" + m_driveSubsystem.getGyroDegrees());
    
=======
        double motorSpeed = MathUtil.clamp(m_pid.calculate(m_driveSubsystem.getYaw(), m_targetAngle), -0.5, 0.5);

>>>>>>> 5ac609f6fafa0cd26c7dfeb401baa0ef64498b14:src/main/java/frc/robot/commands/RotateDrivestationAbsoluteDegreesCommand.java
        m_driveSubsystem.tankDrive(motorSpeed, -motorSpeed);
    }

    @Override
    public boolean isFinished()
    {
<<<<<<< HEAD:src/main/java/frc/robot/commands/AutonomousTurnCommand.java
       // System.out.println("UNDO RETURN FALSE IN AUTONOMOUSTURNCOMMAND");
        //return false;
         return m_pid.atSetpoint();
=======
        return m_pid.atSetpoint();
>>>>>>> 5ac609f6fafa0cd26c7dfeb401baa0ef64498b14:src/main/java/frc/robot/commands/RotateDrivestationAbsoluteDegreesCommand.java
    }

    @Override
    public void end(boolean interrupted)
    {
        m_driveSubsystem.stopDriveTrain();
    }
}
