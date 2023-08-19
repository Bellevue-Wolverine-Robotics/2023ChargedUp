package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.ModesEnum.Throttles;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ArmSubsystem extends SubsystemBase {
    //private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);
    private CANSparkMax m_neoArmMotor = new CANSparkMax(CANConstants.ARM_NEO_MOTOR, MotorType.kBrushless);
    private RelativeEncoder m_neoArmMotorEncoder =  m_neoArmMotor.getEncoder();
    //private SparkMaxPIDController m_pidController = m_neoArmMotor.getPIDController();s

    private boolean m_safety = true;
    private DigitalInput m_armCalibrationSwitch = new DigitalInput(ArmConstants.kArmCalibrationDIO);

    private SlewRateLimiter m_rateLimiter = new SlewRateLimiter(2);
    
    private double throttleLimit = 1.0;



    public ArmSubsystem() {
        //m_armMotor.configFactoryDefault();
        this.m_neoArmMotor.restoreFactoryDefaults();
        //this.m_neoArmMotor.setNeutralMode(NeutralMode.Brake);
        this.m_neoArmMotor.setIdleMode(IdleMode.kBrake);
        
        this.m_neoArmMotorEncoder.setPosition(0);
        
        // m_armMotor.setSensorPhase(true);

        // m_armMotor.config_kF(0, 0);
        // m_armMotor.config_kP(0, ArmConstants.kP);
        // m_armMotor.config_kI(0, ArmConstants.kI);
        // m_armMotor.config_kD(0, ArmConstants.kD);

    }
    

    
    
    public void rotateArm(double speed) {
        // if(this.m_safety){
        //     if(getArmRotationDegrees() > Constants.PhysicalConstants.STRIKE_GROUND_ANGLE) {
        //         speed = speed < 0.0 ? 0 : speed;                
        //     }
        //     if(getArmRotationDegrees() < Constants.PhysicalConstants.STRIKE_ROBOT_ANGLE) {
        //         speed = speed > 0.0 ? 0 : speed;                
        //     }
        // }

        // System.out.println(speed);
        this.m_neoArmMotor.set(throttleLimit*m_rateLimiter.calculate(speed));
    }

    /*public void setthrottleLimit(double limit){
        this.throttleLimit = limit;
    }*/

    public void setThrottleMode(Throttles throttle){
        switch(throttle){
            case high:
                this.throttleLimit = Constants.ThrottleConstants.ROTATION_PRESET_1;
                break;
            case medium:
                this.throttleLimit = Constants.ThrottleConstants.ROTATION_PRESET_2;
                break;
            case low:
                this.throttleLimit = Constants.ThrottleConstants.ROTATION_PRESET_3;
                break;
        }
        
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
        double rotationDeg = (m_neoArmMotorEncoder.getPosition() * PhysicalConstants.Neo_TO_ARM_RATIO * 360) / PhysicalConstants.NEO_PULSES_PER_ROTATION;

        return rotationDeg;
    }

    public double getArmRotationRadians() {
        
        return Math.toRadians(getArmRotationDegrees());
    }

    public void stopArmMotor()
    {
        m_neoArmMotor.stopMotor();
    }

    public boolean isSwitchClosed()
    {
        return m_armCalibrationSwitch.get();
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Rotation Degrees", getArmRotationDegrees());
        // SmartDashboard.putBoolean("Calibration Switched", m_armCalibrationSwitch.get());
 
    }

}
