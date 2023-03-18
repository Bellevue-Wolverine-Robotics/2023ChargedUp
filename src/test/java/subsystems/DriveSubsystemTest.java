import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.subsystems.DriveSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class DriveSubsystemTest {
  static final double DELTA = 1e-2; // acceptable deviation range

  DriveSubsystem m_driveSubsystem;

  @BeforeEach // this method will run before each test
  void setup() {

    m_driveSubsystem = new DriveSubsystem();
  }
}