package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Istream.IStreamBundle;
import frc.robot.Istream.IStreamBundle.IStreamMode;
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
        // TODO: pass in the joystick to constructor
        driveSubsystem.arcadeDrive(input.getY(1), input.getX(1));   

    }
}
