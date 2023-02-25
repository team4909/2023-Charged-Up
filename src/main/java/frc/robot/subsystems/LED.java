// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private static LED m_instance;
    private AddressableLED m_leds;
    private AddressableLEDBuffer m_ledBuffer;
    private final int m_led_length = 60;

    
    
    private LED() {
        m_leds = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(m_led_length);
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.start();
    }

    public static LED getInstance() {
        if (m_instance == null) {
            m_instance = new LED();
        }
        return m_instance;
    }

    @Override
    public void periodic() {
        // for (int i = 0; i < m_led_length; i++) {
        //     m_ledBuffer.setRGB(i, 0, 255, 0);
        // }
        // m_leds.setData(m_ledBuffer);
    }

    
    public void setLedColor(Color color) {
        for (int i = 0; i < m_led_length; i++) {
            m_ledBuffer.setLED(i, color);
        }
        m_leds.setData(m_ledBuffer);
    }
}
