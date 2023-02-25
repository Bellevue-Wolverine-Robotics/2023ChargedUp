package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
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
    private ArmFeedforward m_feedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);

    public RotateArmAbsoluteRadiansCommand(ArmSubsystem m_armSubsystem, double radians){
        this.m_armSubsystem = m_armSubsystem;
        this.m_targetAngle = radians;

        SmartDashboard.putData("arm rotate pid", m_pid);

        addRequirements(this.m_armSubsystem);
    }

    @Override
    public void execute(){
        double pidTerm = this.m_pid.calculate(m_targetAngle, m_armSubsystem.getArmRotationRadians());
        double ffTerm = m_feedForward.calculate(m_targetAngle + ArmConstants.kInitialAngleOffset, 0);
        
        m_armSubsystem.setArmVoltage(pidTerm);
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
