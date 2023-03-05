// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.Utils.SmartDashboardUtils;

// public class RotateArmHardcoded extends CommandBase {
//     private ArmSubsystem m_armSubsystem;
//     private double m_targetAngle;
//     private boolean m_finishes;

//     private ArmFeedforward m_feedForward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV);

//     public RotateArmHardcoded(ArmSubsystem m_armSubsystem, double radians, boolean finishes){
//         this.m_armSubsystem = m_armSubsystem;
//         this.m_targetAngle = radians;
//         this.m_finishes = finishes;

//         addRequirements(this.m_armSubsystem);
//     }

//     @Override
//     public void execute(){
//         // SmartDashboardUtils.TunablePID(this.getName(), m_pid, ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
        
//         double ffTerm = m_feedForward.calculate(Units.degreesToRadians(m_targetAngle + ArmConstants.kInitialAngleOffset), 0);

//         m_armSubsystem.setArmVoltage(pidTerm + ffTerm);
//     }

//     @Override
//     public boolean isFinished() {
//         if (!m_finishes)
//         {
//             return false;
//         }
        
//         return m_pid.atSetpoint();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_armSubsystem.stopArmMotor();
//     }
// }
