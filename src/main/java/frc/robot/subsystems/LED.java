// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private final int led_length = 60;

    
    
    public LED
        
        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(led_length);
        leds.setLength(ledBuffer.getLength());
        leds.start();
    }

    @Override
    public void periodic() {
        // This method will be calledce per scheduler run
        for (int i = 0; i < led_length; i++) {
            ledBuffer.setRGB(i, 0, 255, 0);
        }
    }

    private void a() {
     
    }
}
