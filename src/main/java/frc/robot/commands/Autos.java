// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */

    public static Command doNothing()
    {
        return new WaitCommand(15);
    }

    public static Command scoreMidNoMove(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmScoringAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true));
    }

    public static Command scoreLowNoMove(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmPickupAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true));
    }

    public static Command oneConeCommunity(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmScoringAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.COMMUNITY),
                new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true))
            // new WaitCommand(0.2),
            // new RotateDrivestationAbsoluteDegreesCommand(driveSubsystem, 180)
        );
    }

    public static Command oneConeCommunityScoreLow(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmPickupAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.COMMUNITY),
                new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true))
            // new WaitCommand(0.2),
            // new RotateDrivestationAbsoluteDegreesCommand(driveSubsystem, 180)
        );
    }

    
    public static Command oneConeChargeStation(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmScoringAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.CHARGE_STATION),
                new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true)),
            new BalanceChargeStationCommand(driveSubsystem)
        );
    }

    public static Command scoreLowChargeStation(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem),
            new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmPickupAngle), true),
            new WaitCommand(0.5),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.CHARGE_STATION),
                new RotateArmAbsoluteRadiansCommand(armSubsystem, Units.degreesToRadians(ArmConstants.kArmHomeAngle), true)),
            new BalanceChargeStationCommand(driveSubsystem)
        );
    }

    public static Command hardCodedOneConeCommunityMidScoring(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(() -> armSubsystem.rotateArm(-0.4)),
            new WaitCommand(2.1),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, -4),
                runOnce(() -> armSubsystem.rotateArm(0.4))),
            new WaitCommand(1.8),
            runOnce(() -> armSubsystem.setArmVoltage(0))
        );
    }

    public static Command hardCodedOneConeBalanceMidScoring(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(() -> armSubsystem.rotateArm(-0.4)),
            new WaitCommand(FieldConstants.MID_SCORING_SECONDS),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.COMMUNITY),
                runOnce(() -> armSubsystem.rotateArm(0.4))),
            new WaitCommand(FieldConstants.MID_SCORING_SECONDS - 0.5),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem)
        );
    }

    public static Command hardCodedOneConeCommunityLowScoring(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(() -> armSubsystem.rotateArm(-0.4)),
            new WaitCommand(FieldConstants.LOW_SCORING_SECONDS),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.COMMUNITY),
                runOnce(() -> armSubsystem.rotateArm(0.4))),
            new WaitCommand(FieldConstants.MID_SCORING_SECONDS - 0.5),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem)
        );
    }

    public static Command hardCodedOneConeChargeBalanceLowScoring(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new SequentialCommandGroup(
            runOnce(() -> armSubsystem.rotateArm(-0.4)),
            new WaitCommand(FieldConstants.LOW_SCORING_SECONDS),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::extendIntake, intakeSubsystem),
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.CHARGE_STATION),
                runOnce(() -> armSubsystem.rotateArm(0.4))),
            new WaitCommand(FieldConstants.MID_SCORING_SECONDS - 0.5),
            runOnce(() -> armSubsystem.setArmVoltage(0)),
            runOnce(intakeSubsystem::retractIntake, intakeSubsystem)
        );
    }

    public static Command brokenArmCommunity(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.COMMUNITY);
    }

    public static Command brokenArmChargeStation(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        return new RelativeStraightDriveCommand(driveSubsystem, FieldConstants.CHARGE_STATION);
    }

    public static Command pathWeaverCommand(DriveSubsystem driveSubsystem)
    {
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.VOLTS,
                    DriveConstants.VOLTS_SECONDS_PER_METER,
                    DriveConstants.VOLTS_SECONDS_SQUARED_PER_METER),
                DriveConstants.DRIVE_KINEMATICS,
                DriveConstants.MAX_VOLTAGE);

        Trajectory trajectory = new Trajectory();

        RamseteCommand ramseteCommand =
            new RamseteCommand(
                trajectory,
                driveSubsystem::getPose,
                new RamseteController(DriveConstants.RAMSETE_B, DriveConstants.RAMSETE_ZETA),
                new SimpleMotorFeedforward(
                    DriveConstants.VOLTS,
                    DriveConstants.VOLTS_SECONDS_PER_METER,
                    DriveConstants.VOLTS_SECONDS_SQUARED_PER_METER),
                DriveConstants.DRIVE_KINEMATICS,
                driveSubsystem::getWheelSpeeds,
                new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
                new PIDController(DriveConstants.KP_DRIVE_VEL, 0, 0),
                // RamseteCommand passes volts to the callback
                driveSubsystem::tankDriveVolts,
                driveSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        driveSubsystem.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
    }

    public static Command FastBalanceChargeStationCommand(DriveSubsystem driveSubsystem){
        return new SequentialCommandGroup(new RelativeStraightDriveCommand(driveSubsystem, (Constants.FieldConstants.WHOLE_PLATFORM_LENGTH_METER/2) + Constants.FieldConstants.RAMP_LENGTH_METERS), new BalanceChargeStationCommand(driveSubsystem));
   
   
    }


    // public static Command oneConeChargeTouch() {
    //     return oneConeCommunity();
    // }
}
