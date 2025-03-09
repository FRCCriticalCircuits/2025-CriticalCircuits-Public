package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Physical;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem instance;

    private AddressableLED leds;

    private AddressableLEDBuffer buf;
    private AddressableLEDBufferView bufElev;

    private LEDPattern pattern;
    private LEDPattern patternBlink;

    private boolean blink;
    private Color color;
    
    public LEDSubsystem() {
        // LED Object
        leds = new AddressableLED(Physical.LED.PWM_PORT);

        // configure buffers
        buf = new AddressableLEDBuffer(Physical.LED.NUM_LEDS);
        leds.setLength(Physical.LED.NUM_LEDS);
        
        // initlize bufferView for LED Strip
        bufElev = buf.createView(0, Physical.LED.NUM_ELEVATOR_LED);

        updateColor(Color.kRed);

        leds.start();
    }

    public static LEDSubsystem getInstance() {
        if (instance == null) instance = new LEDSubsystem();
        return instance;
    }

    
    public static void start() {
        LEDSubsystem.getInstance();
    }

    @Override
    public void periodic() {
        updateColor(this.color);

        leds.setData(buf);
    }

    /**
     * Set color and change scrolling zone color
     * @param majorColor Main color
     * @param scrollColor Scroll color
     */
    public void updateColor(Color majorColor, Color scrollColor) {
        this.color = majorColor;

        pattern = LEDPattern.gradient(
            GradientType.kContinuous, 
            majorColor,
            majorColor,
            majorColor,
            majorColor,
            scrollColor,
            majorColor, 
            majorColor, 
            majorColor, 
            majorColor       
        ).scrollAtRelativeSpeed(
            Percent.per(Second).of(Physical.LED.SCROLL_PERCENT_PER_SEC)
        ).atBrightness(
            Percent.of(Physical.LED.BRIGHTNESS)
        );

        patternBlink = LEDPattern.gradient(
            GradientType.kContinuous, 
            majorColor,
            majorColor,
            majorColor,
            majorColor,
            scrollColor,
            majorColor, 
            majorColor, 
            majorColor, 
            majorColor
        ).blink(
            Seconds.of(Physical.LED.BLINK_TIME_ON), Seconds.of(Physical.LED.BLINK_TIME_OFF)
        ).atBrightness(
            Percent.of(Physical.LED.BRIGHTNESS)
        );

        setBlink(this.blink);
    }

    public void updateColor(Color color) {
        updateColor(color, Color.kWhite);
    }

    public void setBlink(boolean state) {
        this.blink = state;

        // Apply pattern per state
        if (state) {
            patternBlink.applyTo(bufElev);
        } else {
            pattern.applyTo(bufElev);
        }
    }
}