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

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;
    private final int led_length = 138;
    
    public LEDSubsystem() {
        
        leds = new AddressableLED(0);
        ledBuffer = new AddressableLEDBuffer(led_length);
        leds.setLength(ledBuffer.getLength());
        leds.start();
    }

    @Override
    public void periodic() {
        // This method will be calledce per scheduler run
    }

    private void a() {
        switch (key) {
            case value:

                break;

            default:
                break;
        }
    }
}
