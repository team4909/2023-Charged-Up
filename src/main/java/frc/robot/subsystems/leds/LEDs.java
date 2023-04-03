package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  // Elevator: 14 x 6, IntaKe: 13 x 3
  private static LEDs m_instance;
  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledBuffer;
  private final int kledLength = 123;

  private LEDs() {
    m_leds = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(kledLength);
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.start();
  }

  public Command setBreatheColor(Color initialColor) {
    final int kStep = 3;
    var currentColor = new Object() {
      double r, g, b, a;
      boolean hitBrightnessLimit;
    };
    return Commands.run(() -> {
      currentColor.r = (currentColor.a / 256.0) * initialColor.red;
      currentColor.g = (currentColor.a / 256.0) * initialColor.green;
      currentColor.b = (currentColor.a / 256.0) * initialColor.blue;
      final Color c = new Color(currentColor.r, currentColor.g, currentColor.b);
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, c);
      }
      if (currentColor.a == 255) {
        currentColor.hitBrightnessLimit = true;
      }
      if (currentColor.hitBrightnessLimit) {
        currentColor.a -= kStep;
        if (currentColor.a == 0)
          currentColor.hitBrightnessLimit = false;
      } else {
        currentColor.a += 5;
      }
      m_leds.setData(m_ledBuffer);
    }, this)
        .ignoringDisable(true);
  }

  public Command setLedColor(Color color) {
    return Commands.runOnce(() -> {
      for (int i = 0; i < kledLength; i++) {
        m_ledBuffer.setLED(i, color);
      }
      m_leds.setData(m_ledBuffer);
    }, this)
        .andThen(Commands.none().repeatedly())
        .ignoringDisable(true);
  }

  public static LEDs getInstance() {
    return m_instance = (m_instance == null) ? new LEDs() : m_instance;
  }
}
