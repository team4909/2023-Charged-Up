package frc.robot.subsystems.leds;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;

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
  private final double kBrightnessCoeff = 1.03;
  private final int kBrightnessLowerLimit = 30;
  private final int kBrightnessUpperLimit = 40;

  private Function<Color, Color> darken = (Color color) -> {
    return new Color(color.red / kBrightnessCoeff, color.green / kBrightnessCoeff, color.blue / kBrightnessCoeff);
  };

  private Function<Color, Color> lighten = (Color color) -> {
    return new Color(color.red * kBrightnessCoeff, color.green * kBrightnessCoeff, color.blue * kBrightnessCoeff);
  };

  private LEDs() {
    m_leds = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(kledLength);
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.start();
  }

  public Command setBreatheColor(Color initialColor) {
    var currentColor = new Object() {
      Color color = initialColor;
      boolean shouldDarken = true;
    };
    return Commands.run(() -> {
      for (int i = 0; i < kledLength; i++)
        m_ledBuffer.setLED(i, currentColor.color);
      Stream<Double> rgb = Stream.of(currentColor.color.red, currentColor.color.green, currentColor.color.blue);
      if (currentColor.shouldDarken) {
        if (rgb.filter(c -> (c * 255) <= (kBrightnessCoeff) + kBrightnessLowerLimit).count() > 1L)
          currentColor.shouldDarken = false;
        else
          currentColor.color = darken.apply(currentColor.color);
      } else {
        if (rgb.filter(c -> (c * 255) <= (kBrightnessCoeff) + kBrightnessUpperLimit).count() > 1L)
          currentColor.color = lighten.apply(currentColor.color);
        else
          currentColor.shouldDarken = true;

      }

      m_leds.setData(m_ledBuffer);
    }, this)
        .beforeStarting(() -> {
          currentColor.color = initialColor;
          currentColor.shouldDarken = true;
        })
        .ignoringDisable(true);

  }

  public Command setFlashColor(Color initialColor) {
    var currentColor = new Object() {
      Color color = initialColor;
      boolean off = false;
    };
    return Commands.run(() -> {
      for (int i = 0; i < kledLength; i++)
        m_ledBuffer.setLED(i, currentColor.color);

      if (currentColor.off) {
        currentColor.color = Color.kBlack;
        currentColor.off = false;
      } else {
        currentColor.color = initialColor;
        currentColor.off = true;
      }
      m_leds.setData(m_ledBuffer);
    }, this)
        .beforeStarting(() -> currentColor.color = initialColor)
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
