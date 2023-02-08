package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    //USING SPARKMAX

    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.INTAKE_FORWARD_CHANNEL, PneumaticsConstants.INTAKE_REVERSE_CHANNEL);
    //private WPI_TalonSRX m_armMotor = new WPI_TalonSRX(CANConstants.ARM_TALON);
    private CANSparkMax m_armMotor = new CANSparkMax(CANConstants.ARM_SPARKMAX, MotorType.kBrushless);
    private RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

    //private RelativeEncoder m_armEncoder = new RelativeEncoder();


    private DoubleSolenoid.Value EXTEND_ENUM = DoubleSolenoid.Value.kForward;
    private DoubleSolenoid.Value RETRACT_ENUM = DoubleSolenoid.Value.kReverse;

    public IntakeSubsystem(){
        this.m_armMotor.restoreFactoryDefaults();
        this.m_armMotor.setIdleMode(IdleMode.kBrake);

        this.m_armEncoder.setPosition(0);

    }
    public void resetPose() {
        //System.out.println("intakeSubsystem.resetPose");
        m_armEncoder.setPosition(0);
       // System.out.println("intakeSubsystem.resetPose Finished");
    }

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
        //not working when during auton when piston is retracted, return kOff, even after setting it to kForward or kBackword.
         
        // this is needed
        System.out.println("toggling sh#t" + m_intakeSolenoid.get());

        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        m_intakeSolenoid.set(toggledValue);

        System.out.println("toggling sh#t" + m_intakeSolenoid.get() + " -----"  + toggledValue);
        /*for (int i = 0; i < 100; i++) {
            System.out.println(m_intakeSolenoid.get());
        }*/

    }

    public void rotateArm(double speed) {
        if(speed < -0.5 || speed > 0.5){//saftey for testing delete later
            System.out.println("@ rotate arm over half power lol");
            m_armMotor.set(speed % 0.5);

            return;
        }

        //System.out.println(getArmRotation());//testing delete later
        m_armMotor.set(speed);
    }

    public double getArmRotation() {
        //returns in radians, does not revert to 0 once pass 2 pi
        //System.out.println(this.m_armEncoder.getPosition()); use later
        return this.m_armEncoder.getPosition()*(2*Math.PI);
    }

    public void resetArmEncoder() {
        this.m_armEncoder.setPosition(0);
    }
}


 
