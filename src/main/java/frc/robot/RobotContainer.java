// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Istream.IStreamBundle;
import frc.robot.Istream.JoysticksStream;
import frc.robot.Istream.XboxStream;
import frc.robot.Istream.IStreamBundle.IStreamMode;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.RelativeStraightDriveCommand;
import frc.robot.commands.AutonomousTurnCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceChargeStation;
import frc.robot.commands.Grab;
import frc.robot.commands.GrabRotateCommand;
import frc.robot.commands.RotateArmAbsoluteRadiansCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final JoysticksStream m_joysticksStream = new JoysticksStream();
  private final XboxStream m_xboxStream = new XboxStream();
  private final IStreamBundle istream = new IStreamBundle(m_xboxStream, m_joysticksStream, IStreamMode.JoysticksMode);

  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 


  public IStreamBundle GetIStream(){
    return this.istream;
  }
//GrabRotateCommand
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandJoystick m_operatorController =
      new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
  //  */
  
  private void configureDefaultCommands()
  {
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(istream, m_driveSubsystem));
    m_intakeSubsystem.setDefaultCommand(new GrabRotateCommand(m_intakeSubsystem, istream));
  }

  private void configureBindings() {
    // m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onTrue(new IntakeGrabCommand(m_intakeSubsystem));
    // m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onFalse(new IntakeReleaseCommand(m_intakeSubsystem));
    m_operatorController.button(ButtonConstants.RESET_POSE_BUTTON).onTrue(new InstantCommand(m_driveSubsystem::resetPose, m_driveSubsystem));

    m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onTrue(new InstantCommand(m_intakeSubsystem::toggleIntake, m_intakeSubsystem));
    m_operatorController.button(ButtonConstants.INTAKE_EXTEND_BUTTON).onTrue(new InstantCommand(m_intakeSubsystem::extendIntake, m_intakeSubsystem));
    m_operatorController.button(ButtonConstants.INTAKE_RETRACT_BUTTON).onTrue(new InstantCommand(m_intakeSubsystem::retractIntake, m_intakeSubsystem));
    m_operatorController.button(ButtonConstants.GRAB_CLOCKWISE_BUTTON).whileTrue(new Grab(m_intakeSubsystem, Grab.direction.CLOCKWISE));
    m_operatorController.button(ButtonConstants.GRAB_COUNTER_CLOCKWISE_BUTTON).whileTrue(new Grab(m_intakeSubsystem, Grab.direction.COUNTERCLOCKWISE));
  
    m_driverController.button(ButtonConstants.ARM_HIGH_BUTTON).onTrue(new RotateArmAbsoluteRadiansCommand(m_intakeSubsystem, Math.PI));
    m_driverController.button(ButtonConstants.ARM_LOW_BUTTON).onTrue(new RotateArmAbsoluteRadiansCommand(m_intakeSubsystem, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String command) {
    return new BalanceChargeStation(m_driveSubsystem);

    // if (command.equals("OneConeAuto"))
    // {
    //   return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem);
    // }
    // else if (command.equals("OneConeCharge"))
    // {
    //   return new RelativeStraightDriveCommand(m_driveSubsystem, -1);
    // }
    // else if (command.equals("ChargeStation")){
    //   return new BalanceChargeStation(m_driveSubsystem);
    // }
    // else {
    //   return new RotateArmAbsoluteRadiansCommand(m_intakeSubsystem, Math.PI);
    // }
    // return new RelativeStraightDriveCommand(m_driveSubsystem, 10);
    // return new RotateArmAbsoluteRadiansCommand(m_intakeSubsystem, Math.PI);
  }

  public void onTeleop() {
    m_intakeSubsystem.onTeleop();
  }
}
