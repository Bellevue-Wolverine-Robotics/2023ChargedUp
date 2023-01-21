package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.INTAKE_FORWARD_CHANNEL, PneumaticsConstants.INTAKE_REVERSE_CHANNEL);
    
    private DoubleSolenoid.Value EXTEND_ENUM = DoubleSolenoid.Value.kReverse;
    private DoubleSolenoid.Value RETRACT_ENUM = DoubleSolenoid.Value.kForward;

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

        System.out.println("Before: " + m_intakeSolenoid.get());
        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        m_intakeSolenoid.set(toggledValue);
        System.out.println(toggledValue);
    }
}



