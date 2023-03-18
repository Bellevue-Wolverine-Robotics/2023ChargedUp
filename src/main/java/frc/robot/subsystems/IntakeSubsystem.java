package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, IntakeConstants.INTAKE_FORWARD_CHANNEL, IntakeConstants.INTAKE_REVERSE_CHANNEL);
    // private DigitalInput m_itemContactSwitch = new DigitalInput(IntakeConstants.kItemContactDIO);

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
        DoubleSolenoid.Value toggledValue = m_intakeSolenoid.get() == DoubleSolenoid.Value.kForward ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward;

        // System.out.println("Before: " + m_intakeSolenoid.get());
        // System.out.println("After: " + toggledValue);
        m_intakeSolenoid.set(toggledValue);
    }
}
