package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private LEDPattern currentPattern;

    public LEDSubsystem() {
        m_led = new AddressableLED(9); // PWM port 0
        m_ledBuffer = new AddressableLEDBuffer(60); // 60 LEDs
        m_led.setLength(m_ledBuffer.getLength());

        // Initialize LED strip to off using pattern
        setPattern(LEDPattern.solid(Color.kGreen));
        m_led.start();
    }

    /**
     * Sets an LED pattern for the entire strip
     * 
     * @param pattern The LEDPattern to display (use LEDPattern factory methods)
     */
    public void setPattern(LEDPattern pattern) {
        currentPattern = pattern;
        pattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /**
     * Sets a solid color for all LEDs
     * 
     * @param color The color to set (use Color constants from WPILib)
     */
    public void setColor(Color color) {
        setPattern(LEDPattern.solid(color));
    }

    @Override
    public void periodic() {
        // Keep LED strip updated with current pattern
        if (currentPattern != null) {
            currentPattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }
    }
}