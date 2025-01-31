// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LedStrip {
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledbuffer;
    public AddressableLEDBufferView m_view1;
    public AddressableLEDSim m_LedSim;
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    LEDPattern yellow = LEDPattern.solid(Color.kYellow);
    LEDPattern off = LEDPattern.solid(Color.kBlack);

    public LedStrip() {
        m_led = new AddressableLED(0);

        m_ledbuffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_ledbuffer.getLength());
        m_view1 = new AddressableLEDBufferView(m_ledbuffer, 30, 59);
        m_led.setData(m_ledbuffer);
        m_led.start();

        m_LedSim = new AddressableLEDSim(m_led);
        m_LedSim.setRunning(true);

    }

    public void setViewOneColor(int reefZone) {
        switch (reefZone) {
            case 0:
                off.applyTo(m_view1);
                break;
            case 1:
            case 4:
                red.applyTo(m_view1);
                break;
            case 3:
            case 5:
                green.applyTo(m_view1);
                break;
            case 2:
            case 6:
                yellow.applyTo(m_view1);
                break;
            default:
                off.applyTo(m_view1);
                break;
        }
        m_led.setData(m_ledbuffer);

    }

}
