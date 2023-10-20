package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax m_neoArmMotor = new CANSparkMax(CANConstants.ARM_NEO_MOTOR, MotorType.kBrushless);
    private RelativeEncoder m_neoArmMotorEncoder =  m_neoArmMotor.getEncoder();
    private boolean m_safety = true;
    private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(2);

    public ArmSubsystem() {
        this.m_neoArmMotor.restoreFactoryDefaults();
        this.m_neoArmMotor.setIdleMode(IdleMode.kBrake);
        this.m_neoArmMotor.setSmartCurrentLimit(8);

        this.m_neoArmMotorEncoder.setPosition(0);
    }
    
    public void rotateArm(double speed) {
        System.out.println("Rotate arm @ " + speed);

        // if(this.m_safety){
        //     if(getArmRotationDegrees() > Constants.PhysicalConstants.STRIKE_GROUND_ANGLE) {
        //         speed = speed < 0.0 ? 0 : speed;                
        //     }
        //     if(getArmRotationDegrees() < Constants.PhysicalConstants.STRIKE_ROBOT_ANGLE) {
        //         speed = speed > 0.0 ? 0 : speed;                
        //     }
        // }

        this.m_neoArmMotor.set(m_rateLimiter.calculate(speed));
    }

    public void setArmVoltage(double outputVolts)
    {
        m_neoArmMotor.setVoltage(outputVolts);
    }

    public void toggleSaftey(){
        this.m_safety = !this.m_safety;
    }

    public void resetArmEncoder() {
        this.m_neoArmMotorEncoder.setPosition(0);
    }

    public double getArmRotationDegrees() {
        double tempNeoMotorRotation = -m_neoArmMotorEncoder.getPosition();
        double rotationDeg = (tempNeoMotorRotation * PhysicalConstants.NEO_TO_ARM_DEGREES);       
        return rotationDeg;
    }

    public double getArmRotationRadians() {
        
        return Math.toRadians(getArmRotationDegrees());
    }

    public void stopArmMotor()
    {
        m_neoArmMotor.stopMotor();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Rotation Degrees", getArmRotationDegrees());
    }

}
