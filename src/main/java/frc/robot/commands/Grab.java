package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import java.lang.Enum;

public class Grab extends CommandBase {
    private IntakeSubsystem m_intakeSubsystem;
    private double grab_power;
    
    public static enum direction{
        CLOCKWISE,
        COUNTERCLOCKWISE
    }



    public Grab (IntakeSubsystem intakeSubsystem, direction dir) {
        m_intakeSubsystem = intakeSubsystem;


        switch(dir){
            case CLOCKWISE:
                this.grab_power = 0.5;
                break;
            case COUNTERCLOCKWISE:
                this.grab_power = -0.5;
            default:
                System.out.println("@ Grab.java");

        }
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        m_intakeSubsystem.rotateGrab(this.grab_power);
    }
}
