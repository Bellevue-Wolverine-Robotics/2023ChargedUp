package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateArmAbsoluteRadiansCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private double m_targetAngle;

    private PIDController m_pid = new PIDController(1, 0, 0); 

    public RotateArmAbsoluteRadiansCommand(IntakeSubsystem intakeSubsystem, double radians){
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_targetAngle = radians;

        m_pid.setTolerance(0.1);

        addRequirements(this.m_intakeSubsystem);
    }

    @Override
    public void execute(){
        double motorSpeed = this.m_pid.calculate(m_intakeSubsystem.getArmRotationRadians(), this.m_targetAngle);
        
        System.out.println("Rotate Arm at: " + motorSpeed);
        motorSpeed = MathUtil.clamp(motorSpeed, -0.5, 0.5);
        m_intakeSubsystem.rotateArm(motorSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopArmMotor();
    }
}
