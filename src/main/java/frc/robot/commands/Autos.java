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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;





public final class Autos {
  /** Example static factory for an autonomous command. */

    /*  not needed for now if defining all commands in constructor
    private DriveSubsystem m_driveSubsystem;
    private IntakeSubsystem m_intakeSubsystem;

    private SequentialCommandGroup[] m_sequentialCommandGroups;
    private ParallelCommandGroup[] m_parallelCommandGroups;
    private ParallelRaceGroup[] m_parallelRaceGroups;
    private ParallelDeadlineGroup[] m_parallelDeadlineGroups;8/
    */
    /*  add back if storing the diff types command in seperate arrays
    public Command getSequentialCommands(int ID) {
        return m_sequentialCommandGroups[ID];
    }

    public Command getParallelCommands(int ID) {
        return m_parallelCommandGroups[ID];
    }

    public Command getParallelRaceCommands(int ID) {
        return m_parallelRaceGroups[ID];
    }

    public Command getParallelDeadlineCommands(int ID) {
        return m_parallelDeadlineGroups[ID];
    }*/

    private Command[] m_commands;
    //private ParallelCommandGroup testCommand;
    public Command runCommand(int ID){
        return m_commands[ID];
    }

   /* public Command testCommand(){
        return this.testCommand;

    }*/

    public Autos(final DriveSubsystem driveSubsystem, final IntakeSubsystem intakeSubsystem) {

        this.m_commands = new Command[5];
        this.m_commands[0] = new SequentialCommandGroup(new IntakeToggleCommand(intakeSubsystem), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));
        this.m_commands[1] = new ParallelCommandGroup(new IntakeToggleCommand(intakeSubsystem), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));
        this.m_commands[2] = new ParallelRaceGroup(new IntakeToggleCommand(intakeSubsystem), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));
        this.m_commands[3] = new ParallelDeadlineGroup(new IntakeToggleCommand(intakeSubsystem), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));
        this.m_commands[4] = new ParallelCommandGroup(new RotateArmAbsoluteCommand(intakeSubsystem, 10*Math.PI), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));
        //this.testCommand = new ParallelCommandGroup(new RotateArmAbsoluteCommand(intakeSubsystem, 4*Math.PI), new AutonomousStraightDriveCommand(driveSubsystem, 6.9));

    

        // test
        /* add back if storing the diff types command in seperate arrays
        m_sequentialCommandGroups = new SequentialCommandGroup[1];
        m_sequentialCommandGroups[0] = new SequentialCommandGroup(new IntakeExtendCommand(m_intakeSubsystem), new AutonomousStraightDriveCommand(m_driveSubsystem, 6.9));
    

        m_parallelCommandGroups = new ParallelCommandGroup[1];
        m_parallelCommandGroups[0] = new ParallelCommandGroup(new IntakeExtendCommand(m_intakeSubsystem), new AutonomousStraightDriveCommand(m_driveSubsystem, 6.9));
    
        m_parallelRaceGroups = new ParallelRaceGroup[1];
        m_parallelRaceGroups[0] = new ParallelRaceGroup(new IntakeExtendCommand(m_intakeSubsystem), new AutonomousStraightDriveCommand(m_driveSubsystem, 6.9));
    
        m_parallelDeadlineGroups = new ParallelDeadlineGroup[1];
        m_parallelDeadlineGroups[0] = new ParallelDeadlineGroup(new IntakeExtendCommand(m_intakeSubsystem), new AutonomousStraightDriveCommand(m_driveSubsystem, 6.9));
        */
    }  
}
