package frc.robot.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardUtils {
    public static void TunablePID(String name, PIDController pidController, double defaultP, double defaultI, double defaultD)
    {
        pidController.setP(SmartDashboard.getNumber(name + " [PID_P]", defaultP));
        pidController.setI(SmartDashboard.getNumber(name + " [PID_I]", defaultI));
        pidController.setD(SmartDashboard.getNumber(name + " [PID_D]", defaultD));

        SmartDashboard.putNumber(name + " [PID_P]", pidController.getP());
        SmartDashboard.putNumber(name + " [PID_I]", pidController.getI());
        SmartDashboard.putNumber(name + " [PID_D]", pidController.getD());

        // System.out.println(name + " CONTROLLER: P(" + pidController.getP() + ") " + "I(" + pidController.getI() + ") " + "D(" + pidController.getD() + ") [[[[SAVE_IN_CONSTANTS]]]]");
    }

    public static void TunableNumber(String name, double defaultNumber)
    {
        
    }
}
