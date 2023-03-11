package frc.robot.commands.teleopDrives;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends CommandBase {
    private DriveSubsystem m_driveSubsystem;
    private DoubleSupplier m_xSpeedSupplier;
    private DoubleSupplier m_zRotSupplier;
    private boolean m_squareInputs;

    public ArcadeDriveCommand(DriveSubsystem drivesystem, DoubleSupplier xSpeedSupplier, DoubleSupplier zRotSupplier, boolean squareInputs){
        this.m_driveSubsystem = drivesystem;

        m_xSpeedSupplier = xSpeedSupplier;
        m_zRotSupplier = zRotSupplier;
        m_squareInputs = squareInputs;

        addRequirements(drivesystem);
    }

    @Override
    public void execute()
    {
        m_driveSubsystem.arcadeDrive(m_xSpeedSupplier.getAsDouble(), m_zRotSupplier.getAsDouble(), m_squareInputs);   
    }
}