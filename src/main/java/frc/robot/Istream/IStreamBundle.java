package frc.robot.Istream;
import frc.robot.Istream.XboxStream;

public class IStreamBundle {
    private JoysticksStream m_joysticks;
    private XboxStream m_xbox;
    private IStreamMode m_mode;


    public IStreamBundle(XboxStream xbox, JoysticksStream joysticks, IStreamMode mode){
        this.m_xbox = xbox;
        this.m_joysticks = joysticks;
    }

    public double getX(int order){
        if(m_mode == IStreamMode.XboxMode){
            //pass
        }

        else if(m_mode == IStreamMode.JoysticksMode){
            try {
                return this.m_joysticks.readX(order);
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        return 0.0;
    }
    
    public double getY(int order){
        if(m_mode == IStreamMode.XboxMode){
            //pass
        }
        if(m_mode == IStreamMode.JoysticksMode){
            try {
                return this.m_joysticks.readY(order);
            } catch (Exception e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        return 0.0;
    }

    
    public void ChangeMode(IStreamMode mode){
        this.m_mode = mode;
    }

    public enum IStreamMode {
        JoysticksMode,
        XboxMode
    }

}
