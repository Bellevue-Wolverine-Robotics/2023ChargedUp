package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RotateArmAbsoluteCommand extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private double m_targetAngle;

    private PIDController m_pid = new PIDController(0.1, 0, 0); 

    public RotateArmAbsoluteCommand(IntakeSubsystem intakeSubsystem, double degrees){
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_targetAngle = degrees;

        m_pid.setTolerance(0.1);

        addRequirements(this.m_intakeSubsystem);
    }

    @Override
    public void execute(){
        double motorSpeed = this.m_pid.calculate(m_intakeSubsystem.getArmRotation(), this.m_targetAngle);
        m_intakeSubsystem.rotateArm(motorSpeed);
    }

    @Override
    public boolean isFinished() {
        return m_pid.atSetpoint();
    }
}
