package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);

    public ArmSubsystem() {
        m_armMotor.configFactoryDefault();
        
        m_armMotor.setSelectedSensorPosition(0);
        m_armMotor.setNeutralMode(NeutralMode.Brake);
        m_armMotor.setSensorPhase(true);
        m_armMotor.setInverted(true);
    }
    
    public void setArmPosition(double pos)
    {
        m_armMotor.set(ControlMode.Position, 0);
    }
    
    public void rotateArm(double speed) {
        m_armMotor.set(speed);
    }

    public void resetArmEncoder() {
        this.m_armMotor.setSelectedSensorPosition(0);
    }

    public double getArmRotationRadians() {
        double rotation = this.m_armMotor.getSelectedSensorPosition() * 2 * Math.PI/ PhysicalConstants.TALON_PULSES_PER_ROTATION;
        
        return rotation;
    }
    
    public void stopArmMotor()
    {
        m_armMotor.stopMotor();
    }
}
