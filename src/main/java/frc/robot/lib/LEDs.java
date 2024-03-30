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
    private int m_numLEDs = 70;
    private int m_r, m_g, m_b;
    private int m_start = 5;
    private int m_midTop = 50;
    private int m_midBot = 20;
    private int[][] map = new int[2][m_numLEDs];
    private int m_meteorCnt = 0;
    private int m_meteorCnt1 = 0;
    private int m_meteorCnt2 = 0;
    private int m_meteorTimeCnt = 0;
    private Color8Bit m_allianceColor1 = null;
    private Color8Bit m_allianceColor2= null;
    private Color8Bit m_allianceColor3 = null;
    private Color8Bit m_allianceColor4 = null;
    private Color8Bit m_allianceColor5 = null;
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
        updateLEDs();
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
        if(m_r != _r || m_g != _g || m_b != _b){
            for (int i = 0; i < m_numLEDs; i++) {
                m_ledBuffer.setRGB(i, _r, _g, _b);
            }
            m_r = _r; m_g = _g; m_b = _b;
            updateLEDs();
        }
        
    }
    public void meteorDown(){

        if(++m_meteorTimeCnt > 5){
            m_meteorTimeCnt = 0;
            
            clearLEDBuffer();
            m_ledBuffer.setLED(m_midTop-m_meteorCnt, m_allianceColor1);
            m_ledBuffer.setLED(m_midTop-m_meteorCnt+1, m_allianceColor2);
            m_ledBuffer.setLED(m_midTop-m_meteorCnt+2, m_allianceColor3);
            m_ledBuffer.setLED(m_midTop-m_meteorCnt+3, m_allianceColor4);
            m_ledBuffer.setLED(m_midTop-m_meteorCnt+4, m_allianceColor5);

            m_ledBuffer.setLED(m_midTop+m_meteorCnt, m_allianceColor1);
            m_ledBuffer.setLED(m_midTop+m_meteorCnt+1, m_allianceColor2);
            m_ledBuffer.setLED(m_midTop+m_meteorCnt+2, m_allianceColor3);
            m_ledBuffer.setLED(m_midTop+m_meteorCnt+3, m_allianceColor4);
            m_ledBuffer.setLED(m_midTop+m_meteorCnt+4, m_allianceColor5);
            if(m_meteorCnt > (m_numLEDs - m_start)/2){

            }
            if(m_meteorCnt > m_numLEDs - m_start){
                m_meteorCnt2 = m_start;
                m_meteorCnt1 = m_midTop-m_meteorCnt;
            }else {
                m_meteorCnt2 = m_midTop+m_meteorCnt;
                m_meteorCnt1 = m_midTop-m_meteorCnt;
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
        if (GD.G_Alliance == Alliance.Blue) {
            setRGBColor(0, 0, 100);
            m_allianceColor1 = new Color8Bit(0, 0, 150);
            m_allianceColor2 = new Color8Bit(0, 0, 120);
            m_allianceColor3 = new Color8Bit(0, 0, 90);
            m_allianceColor4 = new Color8Bit(0, 0, 60);
            m_allianceColor5 = new Color8Bit(0, 0, 30);

        } else {
            setRGBColor(100, 0, 0);
            m_allianceColor1 = new Color8Bit(150, 0, 0);
            m_allianceColor2 = new Color8Bit(120, 0, 0);
            m_allianceColor3 = new Color8Bit(90, 0, 0);
            m_allianceColor4 = new Color8Bit(60, 0, 0);
            m_allianceColor5 = new Color8Bit(30, 0, 0);
        }
    }
    public void clearLEDBuffer(){
        for (int i = 0; i < m_numLEDs; i++){
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        
    }
}