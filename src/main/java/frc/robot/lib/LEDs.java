package frc.robot.lib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDs {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    /**
     * Number of LEDs in the strip. Change this number to match the number of LEDs
     */
    private int m_numLEDs = 48;
    private int ledMidBegin = 16;
    private int ledMidEnd = 32;
    public boolean functionalLEDs = true;

    /**
     * This is a constructor for the class. It has the same name as the class and
     * has no return type.
     *
     * @param _port The PWM port the LEDs are plugged into
     */
    public LEDs(int _port) {
        m_led = new AddressableLED(_port);
        m_ledBuffer = new AddressableLEDBuffer(m_numLEDs);
        m_led.setLength(m_ledBuffer.getLength());
        setAllianceColor();
    }

    public void updateLEDs() {
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Set all the LEDs to the requested RGB values
     *
     * @param _r Red Value between 0-255
     * @param _g Green Value between 0-255
     * @param _b Blue Value between 0-255
     */
    public void setRGBColor(int _r, int _g, int _b) {
        for (int i = 0; i < m_numLEDs; i++) {
            m_ledBuffer.setRGB(i, _r, _g, _b);
        }
    }

    /**
     * Set all the LEDs to the Color8Bit value
     *
     * @param _color The color to set
     */
    public void setRGBColor(Color8Bit _color) {
        setRGBColor(_color.red, _color.green, _color.blue);
    }

    /**
     * Set the LEDs to the color of the alliance the team is on.
     * If the the robot is not connected to the Field Management System (FMS)
     * then the color is green.
     */
    public void setAllianceColor() {
        if (GD.G_Alliance == Alliance.Blue) {
            setRGBColor(0, 0, 100);
        } else {
            setRGBColor(100, 0, 0);
        }
        updateLEDs();
    }

    public void setMiddleGreen() {
        for (int i = ledMidBegin; i < ledMidEnd; i++) {
            m_ledBuffer.setRGB(i, 0, 100, 0);
        }
        updateLEDs();
    }

    public void setMiddleOff() {
        for (int i = ledMidBegin; i < ledMidEnd; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        updateLEDs();
    }

    public void setMiddleAlliance() {
        for (int i = ledMidBegin; i < ledMidEnd; i++) {
            if (GD.G_Alliance == Alliance.Blue) {
                m_ledBuffer.setRGB(i, 0, 0, 100);
            } else {
                m_ledBuffer.setRGB(i, 100, 0, 0);
            }
        }
    }


}