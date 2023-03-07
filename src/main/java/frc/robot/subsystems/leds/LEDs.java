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
  private final int kTopStripColumns = 6;
  private final int kFrontStripColumns = 3;

  private LEDs() {
    m_leds = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(kledLength);
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.start();
  }

  public Command setLedColor(Color color) {
    return Commands.runOnce(() -> {
      for (int i = 0; i < kledLength; i++) {
        m_ledBuffer.setLED(i, color);
      }
      m_leds.setData(m_ledBuffer);
    }, this).ignoringDisable(true);
  }

  int m_position = 0;

  Color currentColor = Color.kAqua;

  public Command strobe() {

    return Commands.run(() -> {
      for (int i = 0; i < kTopStripColumns; i++) {

        if (m_position / kTopStripColumns >= (kledLength / kTopStripColumns) / 2) {
          currentColor = currentColor.equals(Color.kAqua) ? Color.kRed : Color.kAqua;
          m_position = 0;
        }
        m_ledBuffer.setLED(m_position + i, currentColor);
      }

      m_position += kTopStripColumns;
      m_position %= (kledLength - kTopStripColumns);
      m_leds.setData(m_ledBuffer);
    }, this).ignoringDisable(true);
  }

  public static LEDs getInstance() {
    return m_instance = (m_instance == null) ? new LEDs() : m_instance;
  }
}
