package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);
    private boolean m_safety = true;
    private DigitalInput m_armCalibrationSwitch = new DigitalInput(ArmConstants.kArmCalibrationDIO);

    public ArmSubsystem() {
        m_armMotor.configFactoryDefault();
        
        m_armMotor.setSelectedSensorPosition(0);
        m_armMotor.setNeutralMode(NeutralMode.Brake);
        // m_armMotor.setSensorPhase(true);

        // m_armMotor.config_kF(0, 0);
        // m_armMotor.config_kP(0, ArmConstants.kP);
        // m_armMotor.config_kI(0, ArmConstants.kI);
        // m_armMotor.config_kD(0, ArmConstants.kD);

        SmartDashboard.putData("Reset Arm Position", runOnce(this::resetArmEncoder));
    }
    
    public void setArmPosition(double pos)
    {
        m_armMotor.set(ControlMode.Position, pos);
    }
    
    
    public void rotateArm(double speed) {
        
        if(this.m_safety){
            if(getArmRotationDegrees() > Constants.PhysicalConstants.STRIKE_GROUND_ANGLE) {
                speed = speed < 0.0 ? 0 : speed;                
            }
            if(getArmRotationDegrees() < Constants.PhysicalConstants.STRIKE_ROBOT_ANGLE) {
                speed = speed > 0.0 ? 0 : speed;                
            }
        }

        m_armMotor.set(speed);
    }

    public void setArmVoltage(double outputVolts)
    {
        m_armMotor.setVoltage(outputVolts);
    }

    public void toggleSaftey(){
        this.m_safety = !this.m_safety;
    }

    public void resetArmEncoder() {
        this.m_armMotor.setSelectedSensorPosition(0);
    }

    public double getArmRotationDegrees() {
        double rotationDeg = (m_armMotor.getSelectedSensorPosition() * PhysicalConstants.TALON_TO_ARM_RATIO * 360) / PhysicalConstants.TALON_PULSES_PER_ROTATION;

        return rotationDeg;
    }

    public double getArmRotationRadians() {
        
        return Math.toRadians(getArmRotationDegrees());
    }

    public void stopArmMotor()
    {
        m_armMotor.stopMotor();
    }

    public boolean isSwitchClosed()
    {
        return m_armCalibrationSwitch.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("ArmRotation Degrees", getArmRotationDegrees());
        // SmartDashboard.putBoolean("Calibration Switched", m_armCalibrationSwitch.get());

    }

}
