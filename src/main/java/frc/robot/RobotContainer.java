// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ButtonConstants;
import frc.robot.Constants.ButtonConstantsMatthew;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ThrottleConstants;
import frc.robot.ModesEnum.AutoEnum;
import frc.robot.commands.Autos;
import frc.robot.commands.BalanceChargeStationCommand;
import frc.robot.commands.RotateArmAbsoluteRadiansCommand;
import frc.robot.commands.RotateArmSpeedCommand;
import frc.robot.commands.RotateDrivestationAbsoluteDegreesCommand;
import frc.robot.commands.teleopDrives.ArcadeDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import frc.robot.Enums.Throttles;
import frc.robot.Enums.Presets;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); 
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();


  private SendableChooser<Throttles> throttleSelection;
  private SendableChooser<Presets> presetSelection;

  private Throttles prevThrottle;
  private Presets prevPresets;

//GrabRotateCommand
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandJoystick m_driverController =
      new CommandJoystick(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandJoystick m_operatorController =
      new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureSmartDashboardCommands();

    configureDefaultCommands();
    configureBindings();

  }

  private void configureSmartDashboardCommands()
  {
    // ONLY WORK IN TELEOP
    SmartDashboard.putData("Reset Drive Pose", new InstantCommand(m_driveSubsystem::resetPose, m_driveSubsystem));
    SmartDashboard.putData("Reset Arm Position", new InstantCommand(m_armSubsystem::resetArmEncoder, m_armSubsystem));



    throttleSelection = new SendableChooser<Throttles> ();
    throttleSelection.setDefaultOption("Fast", Throttles.FAST);
    throttleSelection.addOption("Medium", Throttles.MEDIUM);
    throttleSelection.addOption("Slow", Throttles.SLOW);

    SmartDashboard.putData("Max Speed", throttleSelection);
    
    /*SmartDashboard.putData("Update Throttle Limit", runOnce(() -> {
       m_driveSubsystem.setThrottleMode(throttleSelection.getSelected()); 
       m_armSubsystem.setThrottleMode(throttleSelection.getSelected()); 

    }, m_driveSubsystem));*/
    this.prevThrottle = throttleSelection.getSelected();




    
    presetSelection = new SendableChooser<Presets> ();
    presetSelection.setDefaultOption("ENABLED", Presets.ENABLED);
    presetSelection.addOption("DISABLED", Presets.DISABLED);  
    SmartDashboard.putData("Arm Presets ", presetSelection);
    this.prevPresets = presetSelection.getSelected();

    
  }

  private void configureDefaultCommands()
  {
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> getArcadeDriveSpeeds().getFirst(), () -> getArcadeDriveSpeeds().getSecond(), false));
  }

  private void configureBindings() {
    // default commands

    BooleanSupplier outsideDeadbandArm = () -> { return Math.abs(m_operatorController.getY()) > OperatorConstants.controllerDeadband;};
    new Trigger(outsideDeadbandArm).whileTrue(new RotateArmSpeedCommand(m_armSubsystem, () -> m_operatorController.getY()*0.4));


    // driving
    m_driverController.button(ButtonConstants.FACE_FORWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 0));
    m_driverController.button(ButtonConstants.FACE_BACKWARDS_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 180));
    m_driverController.button(ButtonConstants.FACE_LEFT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, 90));
    m_driverController.button(ButtonConstants.FACE_RIGHT_BUTTON).whileTrue(new RotateDrivestationAbsoluteDegreesCommand(m_driveSubsystem, -90));

    m_driverController.button(ButtonConstants.RESET_IMU_BUTTON).onTrue(runOnce(m_driveSubsystem::resetImu, m_driveSubsystem));

    m_driverController.button(ButtonConstants.IGNORE_ROTATION_BUTTON).whileTrue(new ArcadeDriveCommand(m_driveSubsystem, () -> -m_driverController.getY(), () -> 0, false));

    m_driverController.button(ButtonConstants.TEST_CHARGE_BALANCE).whileTrue(new BalanceChargeStationCommand(m_driveSubsystem));

    // OPERATORS
    configureOperator();

  }





  public void configureOperator(){     
    m_operatorController.button(ButtonConstants.INTAKE_TOGGLE_BUTTON).onTrue(runOnce(m_intakeSubsystem::toggleIntake, m_intakeSubsystem));

    if (prevPresets == Presets.ENABLED){
  
      m_operatorController.button(ButtonConstants.kHomePositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
      m_operatorController.button(ButtonConstants.kPickupPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
      m_operatorController.button(ButtonConstants.kScoringPositionButton).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
     
      m_operatorController.button(ButtonConstants.kHomePositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmHomeAngle), false));
      m_operatorController.button(ButtonConstants.kPickupPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmPickupAngle), false));
      m_operatorController.button(ButtonConstants.kScoringPositionButtonLH).onTrue(new RotateArmAbsoluteRadiansCommand(m_armSubsystem, Math.toRadians(ArmConstants.kArmScoringAngle), false));
    }
    else{
      m_operatorController.button(ButtonConstants.kHomePositionButton).onTrue(runOnce(() -> {}));
      m_operatorController.button(ButtonConstants.kPickupPositionButton).onTrue(runOnce(() -> {}));
      m_operatorController.button(ButtonConstants.kScoringPositionButton).onTrue(runOnce(() -> {}));
     
      m_operatorController.button(ButtonConstants.kHomePositionButtonLH).onTrue(runOnce(() -> {}));
      m_operatorController.button(ButtonConstants.kPickupPositionButtonLH).onTrue(runOnce(() -> {}));
      m_operatorController.button(ButtonConstants.kScoringPositionButtonLH).onTrue(runOnce(() -> {}));
    }

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
      case LEAVE_COMMUNITY_ONLY:
        return Autos.brokenArmCommunity(m_driveSubsystem, m_intakeSubsystem, m_armSubsystem);


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
 
  /**
   * Returns arcade drive speeds based on throttles and squaring selected
   * @return A pair of the x speed and the z rotation
   */
  private Pair<Double, Double> getArcadeDriveSpeeds()
  {
    double xSpeed = -m_driverController.getY();
    double zRotation = -m_driverController.getX();

    if (DriverStation.getStickButton(OperatorConstants.DRIVER_CONTROLLER_PORT, ButtonConstants.DRIVE_PRESET_3))
    {
      xSpeed *= ThrottleConstants.THROTTLE_PRESET_3;
      zRotation *= ThrottleConstants.ROTATION_PRESET_3;
    }
    else if (DriverStation.getStickButton(OperatorConstants.DRIVER_CONTROLLER_PORT, ButtonConstants.DRIVE_PRESET_2))
    {
      xSpeed *= ThrottleConstants.THROTTLE_PRESET_2;
      zRotation *= ThrottleConstants.ROTATION_PRESET_2;
    }
    else
    {
      xSpeed *= ThrottleConstants.THROTTLE_PRESET_1;
      zRotation *= ThrottleConstants.ROTATION_PRESET_1;
    }
    
    Pair<Double, Double> arcadeDriveSpeedsPair = new Pair<Double, Double>(xSpeed, zRotation);
    
    return arcadeDriveSpeedsPair;
 }

 public void onTeleop() {
  //only gets called once
  new Thread(() ->{
    while(true){
        if(throttleSelection.getSelected() != prevThrottle){
          prevThrottle = throttleSelection.getSelected();

          m_driveSubsystem.setThrottleMode(throttleSelection.getSelected()); 
          m_armSubsystem.setThrottleMode(throttleSelection.getSelected()); 

          // m_intakeSubsystem.setThrottleMode(throttleSelection.getSelected()); 
          //TODO: Maybe Armsubsystem throttle
        }
        if(presetSelection.getSelected() != prevPresets){
          prevPresets = presetSelection.getSelected();
          configureOperator();

        }
    }
       
  }).start();;
  }


  
  public void resetRobotState()
  {
    m_driveSubsystem.resetPose();
    m_armSubsystem.resetArmEncoder();
  }
}

