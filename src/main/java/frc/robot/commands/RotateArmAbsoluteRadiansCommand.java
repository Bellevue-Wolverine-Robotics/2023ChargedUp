package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateArmAbsoluteRadiansCommand extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double m_targetAngle;

    private PIDController m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD); 

    public RotateArmAbsoluteRadiansCommand(ArmSubsystem m_armSubsystem, double radians){
        this.m_armSubsystem = m_armSubsystem;
        this.m_targetAngle = radians;

        m_pid.setTolerance(0.1);
        SmartDashboard.putData("arm rotate pid", m_pid);

        addRequirements(this.m_armSubsystem);
    }

    @Override
    public void execute(){
        double motorSpeed = this.m_pid.calculate(m_armSubsystem.getArmRotationRadians(), this.m_targetAngle);
        
        motorSpeed = MathUtil.clamp(motorSpeed, -0.5, 0.5);
        m_armSubsystem.rotateArm(motorSpeed);
    }

    // @Override
    // public boolean isFinished() {
    //     return m_pid.atSetpoint();
    // }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArmMotor();
    }
}
