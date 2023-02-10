package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    //USING TALON, does not have encoder
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.INTAKE_FORWARD_CHANNEL, PneumaticsConstants.INTAKE_REVERSE_CHANNEL);
    private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);


    public IntakeSubsystem(){
        this.m_armMotor.configFactoryDefault();
        this.m_armMotor.setSelectedSensorPosition(0);
        this.m_armMotor.setNeutralMode(NeutralMode.Brake);
        m_armMotor.setSensorPhase(true);
        m_armMotor.setInverted(true);
    }


    public void extendIntake()
    {
        m_intakeSolenoid.set(PneumaticsConstants.EXTEND_STATE);
    }

    public void retractIntake()
    {
        m_intakeSolenoid.set(PneumaticsConstants.RETRACT_STATE);
    }

    public void toggleIntake()
    {
        // TODO: need to handle initial undetermined state of solenoid

        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        System.out.println("Before: " + m_intakeSolenoid.get());
        System.out.println("After: " + toggledValue);
        m_intakeSolenoid.set(toggledValue);
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

    @Override
    public void periodic()
    {
        // System.out.println("Arm Rotation: " + getArmRotationRadians());
    }

    public void stopArmMotor()
    {
        m_armMotor.stopMotor();
    }

}
