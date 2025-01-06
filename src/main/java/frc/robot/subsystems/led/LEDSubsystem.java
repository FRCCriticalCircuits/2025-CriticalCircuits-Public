package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private static LEDSubsystem instance;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;

    private LEDPattern m_pattern = LEDPattern
    .solid(
        Colors.white
    )
    .atBrightness(
        Percent.of(100)
    )
    .blink(
        Seconds.of(0.15)
    );

    private LEDSubsystem(){
        /* Create Instances */
        m_led = new AddressableLED(9);
        m_buffer = new AddressableLEDBuffer(60);
        m_led.setLength(m_buffer.getLength());

        /* Write Buffer */
        m_led.setData(m_buffer);
        
        /* Enable LED */
        m_led.start();
    }

    public static synchronized LEDSubsystem getInstance(){
        if(instance == null) instance = new LEDSubsystem();
        return instance;
    }

    @Override
    public void periodic() {
        m_pattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
}
