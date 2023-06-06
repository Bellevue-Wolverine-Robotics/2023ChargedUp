// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ButtonConstantsMatthew;
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
import frc.robot.commands.RotateArmAbsoluteRadiansCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveSubsystem.throttles;

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
    // configureBindingsMatthew();

    configureSmartDashboardCommands();
  }

  private void configureSmartDashboardCommands()
  {
    // ONLY WORK IN TELEOP
    SmartDashboard.putData("Reset Drive Pose", new InstantCommand(m_driveSubsystem::resetPose, m_driveSubsystem));
    SmartDashboard.putData("Reset Arm Position", new InstantCommand(m_armSubsystem::resetArmEncoder, m_armSubsystem));
    //SmartDashboard.putData("Set throttle mode", new runOnce(m_driveSubsystem::setThrottleMode, m_driveSubsystem, DriveSubsystem.throttles.high));
    //SmartDashboard.putData("Set throttle mode high", runOnce(() -> {setHigh();}));
   // SmartDashboard.putData("Set throttle mode high 1", runOnce(() -> {setHigh();}));
    // SmartDashboard.putData("Set throttle mode high",  runOnce(() -> { m_driveSubsystem.setThrottleMode(throttles.high); System.out.println("setting high"); }, m_driveSubsystem));
    // SmartDashboard.putData("Set throttle mode medium",  runOnce(() -> { m_driveSubsystem.setThrottleMode(throttles.medium); }, m_driveSubsystem));
    // SmartDashboard.putData("Set throttle mode low", runOnce(() -> { m_driveSubsystem.setThrottleMode(throttles.low); }, m_driveSubsystem));
    
    SendableChooser<throttles> throttleSelection = new SendableChooser<throttles> ();
    throttleSelection.setDefaultOption("fast", throttles.high);
    throttleSelection.addOption("med", throttles.medium);
    throttleSelection.addOption("slow", throttles.low);
    SmartDashboard.putData("Max Speed", throttleSelection);
    SmartDashboard.putData("Update Throttle Limit", runOnce(() -> { m_driveSubsystem.setThrottleMode(throttleSelection.getSelected()); }, m_driveSubsystem));

    m_driveSubsystem.setThrottleMode(throttleSelection.getSelected());
  }
  private void setHigh(){
    runOnce(() -> { m_driveSubsystem.setThrottleMode(throttles.high); }, m_driveSubsystem);
  }
  private void setMiddle(){

  }
  private void setLow(){
    
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
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX(), false));
  }

  private void configureBindings() {
    // default commands

    BooleanSupplier outsideDeadbandArm = () -> { return Math.abs(m_operatorController.getY()) > OperatorConstants.controllerDeadband;};
    new Trigger(outsideDeadbandArm).whileTrue(new RotateArmSpeedCommand(m_armSubsystem, () -> m_operatorController.getY()));

    // driving

    m_driverController.button(ButtonConstants.DRIVE_PRESET_2).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX(), false));
    m_driverController.button(ButtonConstants.DRIVE_PRESET_3).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX(), false));

    m_driverController.button(ButtonConstants.FACE_FORWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 0));
    m_driverController.button(ButtonConstants.FACE_BACKWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 180));
    m_driverController.button(ButtonConstants.FACE_LEFT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 90));
    m_driverController.button(ButtonConstants.FACE_RIGHT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, -90));

    m_driverController.button(ButtonConstants.RESET_IMU_BUTTON).onTrue(runOnce(m_driveSubsystem::resetImu, m_driveSubsystem));

    m_driverController.button(ButtonConstants.IGNORE_ROTATION_BUTTON).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> 0, false));

    m_driverController.button(ButtonConstants.TEST_CHARGE_BALANCE).whileTrue(new BalanceChargeStationCommand(m_driveSubsystem));

    // OPERATORS
    m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onTrue(runOnce(m_intakeSubsystem::toggleIntake, m_intakeSubsystem));
  
    m_operatorController.button(ButtonConstants.kHomePositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstants.kPickupPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstants.kScoringPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
   
    m_operatorController.button(ButtonConstants.kHomePositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstants.kPickupPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstants.kScoringPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
  
  }

  public void configureBindingsMatthew()
  {
    BooleanSupplier outsideDeadbandArm = () -> { return Math.abs(m_operatorController.getY()) > OperatorConstants.controllerDeadband;};
    new Trigger(outsideDeadbandArm).whileTrue(new RotateArmSpeedCommand(m_armSubsystem, () -> m_operatorController.getY()));

    // driving

    m_driverController.button(ButtonConstantsMatthew.DRIVE_PRESET_2).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX(), false));
    m_driverController.button(ButtonConstantsMatthew.DRIVE_PRESET_3).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> -m_driverController.getX(), false));

    // m_driverController.button(ButtonConstants.FACE_FORWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 0));
    // m_driverController.button(ButtonConstants.FACE_BACKWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 180));
    // m_driverController.button(ButtonConstants.FACE_LEFT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 90));
    // m_driverController.button(ButtonConstants.FACE_RIGHT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, -90));

    // m_driverController.button(ButtonConstants.RESET_IMU_BUTTON).onTrue(runOnce(m_driveSubsystem::resetImu, m_driveSubsystem));

    m_driverController.button(ButtonConstantsMatthew.IGNORE_ROTATION_BUTTON).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> 0, false));

    m_driverController.button(ButtonConstantsMatthew.TEST_CHARGE_BALANCE_BUTTON).whileTrue(new BalanceChargeStationCommand(m_driveSubsystem));

    // OPERATORS
    m_operatorController.button(ButtonConstantsMatthew.INTAKE_TOGGLE_BUTTON).onTrue(runOnce(m_intakeSubsystem::toggleIntake, m_intakeSubsystem));
  
    m_operatorController.button(ButtonConstantsMatthew.kHomePositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstantsMatthew.kPickupPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstantsMatthew.kScoringPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));

    m_operatorController.button(ButtonConstantsMatthew.kHomePositionButtonAlt).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
    m_operatorController.button(ButtonConstantsMatthew.kPickupPositionButtonAlt).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
    m_operatorController.button(ButtonConstantsMatthew.kScoringPositionButtonAlt).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));

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
        return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case ONE_CONE_BALANCE_CHARGE_STATION:
        return Autos.oneConeChargeStation(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case BALANCE_ONLY:
        return Autos.FastBalanceChargeStationCommand(m_driveSubsystem);
      case ONE_CONE_SCORE_LOW_LEAVE_COMMUNITY:
        return Autos.oneConeCommunityScoreLow(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case DO_NOTHING:
        return Autos.doNothing();
      case SCORE_MID_DO_NOTHING:
        return Autos.scoreMidNoMove(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case SCORE_LOW_DO_NOTHING:
        return Autos.scoreLowNoMove(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
      case ONE_CONE_SCORE_LOW_CHARGE_STATION:
        return Autos.scoreLowChargeStation(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);

    }
    // return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
    // return Autos.brokenArmCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.hardCodedOneConeCommunityMidScoring(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.oneConeTouch(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return Autos.brokenArmChargeStation(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
   // return new SequentialCommandGroup(new RelativeStraightDriveCommand(m_driveSubsystem, 10), new AutonomousTurnHardcodeCommand(m_driveSubsystem, 90), new WaitCommand(2), new RelativeStraightDriveCommand(m_driveSubsystem, 5)); 
    return Autos.oneConeCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);
    
    //return new RelativeStraightDriveCommand(m_driveSubsystem, 10);
  }
  
  public void onTeleop() {
  }

  public void onAuton()
  {
    m_driveSubsystem.resetPose();
    m_armSubsystem.resetArmEncoder();
  }

  public ArmSubsystem getArmSubsystem() {
    return m_armSubsystem;
  }
}

