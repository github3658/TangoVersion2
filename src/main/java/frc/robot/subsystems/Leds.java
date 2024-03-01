package frc.robot.subsystems;
    import frc.robot.commands.*;
    import edu.wpi.first.wpilibj.livewindow.LiveWindow;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import com.ctre.phoenix.led.CANdle;
    import com.ctre.phoenix.led.RainbowAnimation;
    import com.ctre.phoenix.led.StrobeAnimation;
    
    
    public class Leds extends SubsystemBase {

public enum Color{
  YELLOW,
  GREEN
};

public enum Pattern{
SOLID,
BLINK,
RAINBOW
};

private final int canID = 20;
private final int numberOfLEDs = 100;
private final int[] yellow = {255, 255, 0};
private final int[] green = {0, 255, 0};


//private final int[] purple = {80, 45, 127};
//private final int[] gold = {255, 200, 46};

private int brightness = 1;
private int speed = 1;

private final CANdle m_candlenew = CANdle(canID);
private final RainbowAnimation m_rainbow = new RainbowAnimation(brightness, speed, numberOfLEDs);
private final StrobeAnimation m_strobe = new StrobeAnimation(255, 0, 0, speed, numberOfLEDs);
private Animation m_currentPattern = null;
private int[] m_currentColor = {255, 255, 0;
       
        public Leds() {
        }
    
        @Override
        public void periodic() {
            // This method will be called once per scheduler run
if (m_animate != null)
{
   m_candle.animate(m_animate);
}
else
{
   m_candle.setLEDs(m_currentColor[0], m_currentColor[1], m_currentColor[2]);
}
    
        }
    
        @Override
        public void simulationPeriodic() {
            // This method will be called once per scheduler run when in simulation
    
        }

public void setColorPattern(Color color, Pattern pattern){
if (color == Color.yellow)
{
   m_currentColor = m_yellow;
}
else
{
   m_currentColor = m_green;
}

if (pattern == Pattern.SOLID)
{
   m_currentPattern = null;
}
else if (pattern == Pattern.BLINK)
{
   m_currentPattern = m_strobe;
m_strobe.setR(m_currentColor[0]);
m_strobe.setG(m_currentColor[1]);
m_strobe.setB(m_currentColor[2]);
}
else
{
m_currentPattern = m_rainbow;
}

}
    }    
