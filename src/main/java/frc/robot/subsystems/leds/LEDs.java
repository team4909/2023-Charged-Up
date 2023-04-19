package frc.robot.subsystems.leds;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  // Elevator: 14 x 6, IntaKe: 13 x 3
  private final int kledLength = 123;
  private static LEDs m_instance;
  private AddressableLED m_leds;
  private AddressableLEDBuffer m_ledBuffer;
  private Color m_currentColor = Color.kWhite;

  private LEDs() {
    m_leds = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(kledLength);
    m_leds.setLength(m_ledBuffer.getLength());
    m_leds.start();
  }

  public Command SetBreatheColor(Color initialColor) {
    final int kStep = 3;
    var currentColor = new Object() {
      double r, g, b, a;
      boolean hitBrightnessLimit;
    };
    return this.run(() -> {
      m_currentColor = initialColor;
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
    })
        .ignoringDisable(true)
        .withName("Set Breathe Color");
  }

  public void setColor(Color color) {
    if (m_currentColor.equals(color))
      return;
    m_currentColor = color;
    for (int i = 0; i < kledLength; i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_leds.setData(m_ledBuffer);
  }

  public Command SetStaticColor(Color color) {
    return this.run(() -> setColor(color))
        .finallyDo((i) -> setColor(Color.kBlack))
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .withName("Set Static Color");
  }

  public Command GamePieceIndicator(DoubleSupplier coneCurrent, DoubleSupplier cubeCurrent) {
    Timer coneStallTimer = new Timer();
    Timer cubeStallTimer = new Timer();
    SmartDashboard.putNumber("cone current", 1);
    SmartDashboard.putNumber("cube current", 1);
    final double kTriggerTime = 0.15;
    return this.run(() -> {
      if (SmartDashboard.getNumber("cone current", 0) >= 40)
        coneStallTimer.start();
      else
        coneStallTimer.stop();

      if (SmartDashboard.getNumber("cube current", 0) >= 15)
        cubeStallTimer.start();
      else
        cubeStallTimer.stop();

      if (coneStallTimer.get() >= kTriggerTime || cubeStallTimer.get() >= kTriggerTime) {
        setColor(Color.kAqua);
      } else {
        // setColor(Color.kBlack);
      }
    }).beforeStarting(() -> {
      coneStallTimer.reset();
      cubeStallTimer.reset();
    })
        .withName("Game Piece Indicator");
  }

  public void periodic() {
    SmartDashboard.putString("Current Color", m_currentColor.toString());
    SmartDashboard.putString("Current Command",
        this.getCurrentCommand() == null ? "Null" : this.getCurrentCommand().getName());
  }

  public static LEDs getInstance() {
    return m_instance = (m_instance == null) ? new LEDs() : m_instance;
  }
}
