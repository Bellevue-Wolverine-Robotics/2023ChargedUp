// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Istream.IStreamBundle;
import frc.robot.Istream.JoysticksStream;
import frc.robot.Istream.XboxStream;
import frc.robot.Istream.IStreamBundle.IStreamMode;
import frc.robot.commands.AutonomousTurnHardcodeCommand;
import frc.robot.commands.RelativeStraightDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceChargeStationCommand;
import frc.robot.commands.InitArmPositionCommand;
import frc.robot.commands.RotateArmSpeedCommand;
import frc.robot.commands.RotateDrivestationAbsoluteDegreesCommand;
import frc.robot.commands.teleopDrives.ArcadeDriveCommand;
import frc.robot.commands.teleopDrives.CurvatureDriveCommand;
import frc.robot.commands.teleopDrives.SemiConstantCurvatureDriveCommand;
import frc.robot.commands.RotateArmAbsoluteRadiansCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.BooleanSupplier;

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
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

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
  }

  private void configureBindings() {
    // default commands

    BooleanSupplier outsideDeadbandArm = () -> { return Math.abs(m_operatorController.getY()) > OperatorConstants.controllerDeadband;};
    new Trigger(outsideDeadbandArm).whileTrue(new RotateArmSpeedCommand(m_armSubsystem, () -> m_operatorController.getY()));

    BooleanSupplier outsideDeadbandDrive = () -> { return Math.abs(m_driverController.getX()) > OperatorConstants.controllerDeadband || Math.abs(m_driverController.getY()) > OperatorConstants.controllerDeadband;};
    new Trigger(outsideDeadbandDrive).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY() * DriveConstants.THROTTLE_PRESET_1, () -> -m_driverController.getX() * DriveConstants.ROTATION_PRESET_1, false));

    // driving

    m_driverController.button(ButtonConstants.DRIVE_PRESET_2).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY() * DriveConstants.THROTTLE_PRESET_2, () -> -m_driverController.getX() * DriveConstants.ROTATION_PRESET_2, false));
    m_driverController.button(ButtonConstants.DRIVE_PRESET_3).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY() * DriveConstants.THROTTLE_PRESET_3, () -> -m_driverController.getX() * DriveConstants.ROTATION_PRESET_3, false));

    // m_driverController.button(ButtonConstants.TEST_CHARGE_BALANCE_BUTTON).whileTrue(new BalanceChargeStationCommand(m_driveSubsystem));    m_driverController.button(ButtonConstants.FACE_FORWARDS_BUTTON).onTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 0));
    m_driverController.button(ButtonConstants.FACE_BACKWARDS_BUTTON).onTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 180));
    m_driverController.button(ButtonConstants.FACE_LEFT_BUTTON).onTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 90));
    m_driverController.button(ButtonConstants.FACE_RIGHT_BUTTON).onTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, -90));

    m_driverController.button(ButtonConstants.RESET_IMU_BUTTON).onTrue(runOnce(m_driveSubsystem::resetImu, m_driveSubsystem));

    // m_driverController.button(8).onTrue(new AutonomousTurnHardcodeCommand(m_driveSubsystem, 360));
    // m_driverController.button(10).onTrue(new AutonomousTurnHardcodeCommand(m_driveSubsystem, 180));
    // m_driverController.button(12).onTrue(new AutonomousTurnHardcodeCommand(m_driveSubsystem, 90))
    
    // OPERATORS
    m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onTrue(runOnce(m_intakeSubsystem::toggleIntake, m_intakeSubsystem));
  
    m_operatorController.button(ButtonConstants.kHomePositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstants.kPickupPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstants.kScoringPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
   
    m_operatorController.button(ButtonConstants.kHomePositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstants.kPickupPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstants.kScoringPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(AutoEnum autoEnum) {
    switch (autoEnum)
    {
      case ONE_CONE_LEAVE_COMMUNITY:
        System.out.println("leave com");
        return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case ONE_CONE_BALANCE_CHARGE_STATION:
        System.out.println("charge station");
        return Autos.oneConeChargeStation(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
    }
    // return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
    // return Autos.brokenArmCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.hardCodedOneConeCommunityMidScoring(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.oneConeTouch(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.brokenArmChargeStation(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
    return new SequentialCommandGroup(new RelativeStraightDriveCommand(m_driveSubsystem, 10), new AutonomousTurnHardcodeCommand(m_driveSubsystem, 90), new WaitCommand(2), new RelativeStraightDriveCommand(m_driveSubsystem, 5)); 
    
    
    //return new RelativeStraightDriveCommand(m_driveSubsystem, 10);
  }
  
  public void onTeleop() {
    m_intakeSubsystem.onTeleop();
  }

  public ArmSubsystem getArmSubsystem() {
    return m_armSubsystem;
  }
}
