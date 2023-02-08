package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystemTalon extends SubsystemBase {
    //USING TALON, does not have encoders
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.INTAKE_FORWARD_CHANNEL, PneumaticsConstants.INTAKE_REVERSE_CHANNEL);
    private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);
    // private CANSparkMax m_armMotor = new CANSparkMax(CANConstants.ARM_SPARKMAX, MotorType.kBrushless);
    // private RelativeEncoder m_armEncoder = new RelativeEncoder();
    private DoubleSolenoid.Value EXTEND_ENUM = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value RETRACT_ENUM = DoubleSolenoid.Value.kReverse;

    public void extendIntake()
    {
        m_intakeSolenoid.set(EXTEND_ENUM);
    }

    public void retractIntake()
    {
        m_intakeSolenoid.set(RETRACT_ENUM);
    }

    public void toggleIntake()
    {
        // TODO: need to handle initial undetermined state of solenoid

        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        m_intakeSolenoid.set(toggledValue);
    }

    public void rotateArm(double speed) {
        m_armMotor.set(speed);
    }

    public double getArmRotation() {
        // until we get arm encoder
        return 0;
    }

    public void resetArmEncoder() {
        
    }
}



