package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class IntakeSubsystem extends SubsystemBase {
    private DoubleSolenoid m_intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PneumaticsConstants.INTAKE_FORWARD_CHANNEL, PneumaticsConstants.INTAKE_REVERSE_CHANNEL);

    public void extendIntake()
    {
        m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void retractIntake()
    {
        m_intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
}



