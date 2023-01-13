package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Istream.IStreamBundle;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private IStreamBundle input;

    public ArcadeDriveCommand(IStreamBundle input, DriveSubsystem drivesystem){
        this.input = input;
        this.driveSubsystem = drivesystem;

        addRequirements(drivesystem);
    }

    @Override
    public void execute()
    {
        System.out.println(" here ^2 at arcade");
        //istream ??
        //driveSubsystem.arcadeDrive(input.getX(1), input.getY(1));   
        //driveSubsystem.arcadeDrive(0.5, 0.5);        
        //drive subsystem 
       // driveSubsystem.arcadeDrive(0.5, 0.5);
       driveSubsystem.tankDrive(0.5, 0.5);        
        
     
    }
}
