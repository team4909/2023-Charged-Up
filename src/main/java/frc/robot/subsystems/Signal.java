package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Signal {
    public Signal() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         
         Signal.setData(m_ledBuffer);
    }

    @Override
    public void robotInit() {
      // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);
  
      // Reuse buffer
      // Default to a length of 60, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(60);
      m_led.setLength(m_ledBuffer.getLength());
  
      // Set the data
      m_led.setData(m_ledBuffer);
      m_led.start();
    }
}


