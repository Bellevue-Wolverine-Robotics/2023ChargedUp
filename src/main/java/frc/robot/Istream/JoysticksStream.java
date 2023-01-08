package frc.robot.Istream;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;;

public class JoysticksStream {
    private final Joystick stick1 = new Joystick(OperatorConstants.JOYSTICK_1);
    private final Joystick stick2 = new Joystick(OperatorConstants.JOYSTICK_2);
    private final Joystick stick3 = new Joystick(OperatorConstants.JOYSTICK_3);


    public JoysticksStream(){

    }

    public double readMagnitude(int id) throws Exception{
        switch(id){
            case 1:
                return this.stick1.getMagnitude();
            case 2:
                return this.stick2.getMagnitude();
            case 3:
                return this.stick3.getMagnitude();
        }

        throw new Exception("Out of bound joystick @ jOYSTICKS.JAVA");
    }
    public double readY(int id) throws Exception{
        switch(id){
            case 1:
                return this.stick1.getY();
            case 2:
                return this.stick2.getY();
            case 3:
                return this.stick3.getY();
        }

        throw new Exception("Out of bound joystick @ jOYSTICKS.JAVA");
    }
    public double readX(int id) throws Exception{
        switch(id){
            case 1:
                return this.stick1.getX();
            case 2:
                return this.stick2.getX();
            case 3:
                return this.stick3.getX();
        }

        throw new Exception("Out of bound joystick @ jOYSTICKS.JAVA");
    }
}
