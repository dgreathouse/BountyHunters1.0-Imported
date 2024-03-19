package frc.robot.lib;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;

import java.util.Random;

public class LEDs {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private Timer m_blinkTimer;
    /**
     * Number of LEDs in the strip. Change this number to match the number of LEDs
     */
    private int m_numLEDs = 50;
    private int m_red = 0;
    private int m_green = 0;
    private int m_blue = 0;
    private boolean m_blinkState = true;
    private Random _random = new Random();
    private int ledMidBeggining = 16;
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
        m_led.setData(m_ledBuffer);
        m_led.start();
        m_blinkTimer = new Timer();
        m_blinkTimer.start();
    }

    public void FunctionalLEDs() {
        functionalLEDs = !functionalLEDs;
    }

    /**
     * Set all the LEDs to the requested RGB values
     * 
     * @param _r Red Value between 0-255
     * @param _g Green Value between 0-255
     * @param _b Blue Value between 0-255
     */
    public void setRGBColor(int _r, int _g, int _b) {
        if (_r == m_red && _g == m_green && _b == m_blue) {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, _r, _g, _b);
            }
            m_led.setData(m_ledBuffer);
            m_led.start();
        }
    }

    public void setLEDsThreePartRGBColor(int _r1, int _g1, int _b1, int _r2, int _g2, int _b2, int _r3, int _g3,
            int _b3) {
        if (_r2 == m_red && _g2 == m_green && _b2 == m_blue) {
            // do nothing
        } else {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                if (i <= ledMidBeggining) {
                    m_ledBuffer.setRGB(i, _r1, _g1, _b1);
                } else if (i <= ledMidEnd && i > ledMidBeggining) {
                    m_red = _r2;
                    m_green = _g2;
                    m_blue = _b2;
                    m_ledBuffer.setRGB(i, m_red, m_green, m_blue);
                } else if (i > ledMidEnd) {
                    m_ledBuffer.setRGB(i, _r3, _g3, _b3);
                }
            }
            m_led.setData(m_ledBuffer);
            m_led.start();
        }

    }

    public void celebration() {
        if (functionalLEDs == false) {
            int rand = _random.nextInt(m_ledBuffer.getLength());
            if (m_blinkTimer.hasElapsed(0.05)) {
                m_blinkState = !m_blinkState;
                m_blinkTimer.restart();
            }
            if (m_blinkState) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    m_ledBuffer.setRGB(rand, 0, 0, 100);
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                    m_ledBuffer.setRGB(rand, 100, 0, 0);
                }
                m_led.setData(m_ledBuffer);
                m_led.start();
            } else {
                setRGBColor(0, 0, 0);
            }
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
        if (functionalLEDs) {
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                setRGBColor(0, 0, 100);
            } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                setRGBColor(100, 0, 0);
            } else {
                setRGBColor(50, 50, 50);
            }
        }
        Robot.DELETE += 1;
    }

    /**
     * Set the color to green if difference between requested and actual are within
     * the tolerance
     * 
     * @param _requested The value that is requested
     * @param _actual    The actual value
     * @param _tolerance The tolerance
     */

    public void setBlinky(double _time, int _r, int _g, int _b) {
        if (functionalLEDs) {
            if (m_blinkTimer.hasElapsed(_time)) {
                m_blinkState = !m_blinkState;
                m_blinkTimer.restart();
            }
            if (m_blinkState) {
                setRGBColor(_r, _g, _b);
            } else {
                setRGBColor(0, 0, 0);
            }
        }
    }

    public void setMulticolorBlinky(double _time, int _r, int _g, int _b) {
        if (functionalLEDs) {
            if (m_blinkTimer.hasElapsed(_time)) {
                m_blinkState = !m_blinkState;
                m_blinkTimer.restart();
            }
            if (m_blinkState) {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    setLEDsThreePartRGBColor(0, 0, 100, _r, _g, _b, 0, 0, 100);
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                    setLEDsThreePartRGBColor(100, 0, 0, _r, _g, _b, 100, 0, 0);
                } else {
                    setRGBColor(_r, _g, _b);
                }
            } else {
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    setLEDsThreePartRGBColor(0, 0, 100, 0, 0, 0, 0, 0, 100);
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                    setLEDsThreePartRGBColor(100, 0, 0, 0, 0, 0, 100, 0, 0);
                } else {
                    setRGBColor(0, 0, 0);
                }
            }
        }
    }
}