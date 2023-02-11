// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.Grab.direction;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;





public final class Autos {
  /** Example static factory for an autonomous command. */

    public static Command oneConeCommunity(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::extendIntake, intakeSubsystem),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, 2),
                new RotateArmAbsoluteRadiansCommand(intakeSubsystem, Math.PI)),
            new WaitCommand(1),
            new InstantCommand(intakeSubsystem::retractIntake, intakeSubsystem),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new RelativeStraightDriveCommand(driveSubsystem, -4),
                new RotateArmAbsoluteRadiansCommand(intakeSubsystem, 0))
            
        );
    }

    // public static Command oneConeChargeTouch() {
    //     return oneConeCommunity();
    // }
}
