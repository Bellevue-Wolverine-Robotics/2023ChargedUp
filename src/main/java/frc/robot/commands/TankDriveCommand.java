package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Istream.IStreamBundle;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class TankDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private IStreamBundle input;
    /*byte shoulderMotorChannel;
    Spark shoulder = new Spark(shoulderMotorChannel);*/
    public TankDriveCommand(DriveSubsystem driveSubsystem, IStreamBundle input){
        this.driveSubsystem = driveSubsystem;
        this.input = input;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute(){
        // shoulder.set()
        
        driveSubsystem.tankDrive(input.getY(1), input.getY(2));
    }
}
