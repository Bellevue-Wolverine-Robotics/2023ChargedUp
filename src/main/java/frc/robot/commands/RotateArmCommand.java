package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.lang.Enum;

public class RotateArmCommand extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private double grab_power;
    
    public static enum direction {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public RotateArmCommand (ArmSubsystem m_armSubsystem, direction dir) {
        this.m_armSubsystem = m_armSubsystem;

        switch(dir){
            case CLOCKWISE:
                this.grab_power = 0.1;
                break;
            case COUNTERCLOCKWISE:
                this.grab_power = -0.1;
            default:
                System.out.println("@ Grab.java");
        }

        addRequirements(m_armSubsystem);
    }

    @Override
    public void execute() {
        m_armSubsystem.rotateArm(this.grab_power);
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.stopArmMotor();
    }
}
