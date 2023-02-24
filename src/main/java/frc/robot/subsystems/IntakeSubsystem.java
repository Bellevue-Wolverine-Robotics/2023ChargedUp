package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    //USING TALON, does not have encoder
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_FORWARD_CHANNEL, IntakeConstants.INTAKE_REVERSE_CHANNEL);
    private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);
    private DigitalInput m_itemContactSwitch = new DigitalInput(9); // TODO: constatn later

    public IntakeSubsystem(){
        this.m_armMotor.configFactoryDefault();
        this.m_armMotor.setSelectedSensorPosition(0);
        this.m_armMotor.setNeutralMode(NeutralMode.Brake);
        m_armMotor.setSensorPhase(true);
        m_armMotor.setInverted(true);

        m_armMotor.selectProfileSlot(0, 0);
        m_armMotor.config_kF(0, 0);
        m_armMotor.config_kP(0, 0.5);
        m_armMotor.config_kI(0, 0);
        m_armMotor.config_kD(0, 0);

    }

    public void extendIntake()
    {
        m_intakeSolenoid.set(IntakeConstants.EXTEND_STATE);
    }

    public void retractIntake()
    {
        m_intakeSolenoid.set(IntakeConstants.RETRACT_STATE);
    }

    public void toggleIntake()
    {
        // TODO: need to handle initial undetermined state of solenoid

        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        System.out.println("Before: " + m_intakeSolenoid.get());
        System.out.println("After: " + toggledValue);
        m_intakeSolenoid.set(toggledValue);
    }

    public void onTeleop()
    {
        new Trigger(m_itemContactSwitch::get).onTrue(runOnce(this::extendIntake));
    }

}
