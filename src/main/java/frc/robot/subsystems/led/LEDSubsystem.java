package frc.robot.subsystems.led;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Physical.LEDSubsystemConstants.*;
import static edu.wpi.first.units.Units.*;

public class LEDSubsystem extends SubsystemBase {
    public static final Color CICRed = new Color("#89191c");
    private static LEDSubsystem instance;

    AddressableLED leds;

    AddressableLEDBuffer buf;
    AddressableLEDBufferView bufElev;
    LEDPattern pattern;
    LEDPattern patternBlink;

    boolean blink;
    Color color;
    
    public LEDSubsystem() {
        leds = new AddressableLED(PWM_PORT);

        // buffer length is expensive to change make it constant
        buf = new AddressableLEDBuffer(NUM_LEDS);
        leds.setLength(NUM_LEDS);
        // elevator leds
        bufElev = buf.createView(0, NUM_LAST_ELEV_LED);

        // pattern = LEDPattern.gradient(
        //     GradientType.kContinuous, 
        //     CICRed, CICRed, Color.kWhite, CICRed, CICRed
        // // Add cycling white zone
        // ).scrollAtRelativeSpeed(Percent.per(Second).of(0.25))
        // .atBrightness(Percent.of(20));

        // // Separate blinking pattern
        // patternBlink = LEDPattern.gradient(
        //     GradientType.kContinuous, 
        //     CICRed, CICRed, Color.kWhite, CICRed, CICRed
        // // Add cycling white zone
        // ).scrollAtRelativeSpeed(Percent.per(Second).of(0.25))
        // .blink(Seconds.of(0.1), Seconds.of(0.05))
        // .atBrightness(Percent.of(20));


        setColor(Color.kRed);
        leds.start();
    }

    @Override
    public void periodic() {
        setColor(this.color);
        leds.setData(buf);
    }

    public void setColor(Color c) {
        color = c;
        pattern = LEDPattern.gradient(
            GradientType.kContinuous, 
            c, c, c, c, Color.kWhite, c, c, c, c
        // Add cycling white zone       
        ).scrollAtRelativeSpeed(Percent.per(Second).of(SCROLL_PERCENT_PER_SEC))
        .atBrightness(Percent.of(BRIGHTNESS));

        patternBlink = LEDPattern.gradient(
            GradientType.kContinuous, 
            c, c, c, c, Color.kWhite, c, c, c, c
        // Add cycling white zone
        ).blink(Seconds.of(BLINK_TIME_ON), Seconds.of(BLINK_TIME_OFF))
        .atBrightness(Percent.of(BRIGHTNESS));

        setBlink(this.blink);
    }

    /**
     * Set color and change scrolling zone color
     * @param c Main color
     * @param c2 Scroll color
     */
    public void setColor(Color c, Color c2) {
        pattern = LEDPattern.gradient(
            GradientType.kContinuous, 
            c, c, c, c,  c2, c, c, c, c
        // Add cycling white zone
        ).atBrightness(Percent.of(BRIGHTNESS))
        .scrollAtRelativeSpeed(Percent.per(Second).of(SCROLL_PERCENT_PER_SEC));

        patternBlink = LEDPattern.gradient(
            GradientType.kContinuous, 
            c, c, c, c, c2, c, c, c, c
        // Add cycling white zone
        ).atBrightness(Percent.of(BRIGHTNESS))
        .blink(Seconds.of(BLINK_TIME_ON), Seconds.of(BLINK_TIME_OFF));

        setBlink(this.blink);
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

    public static LEDSubsystem getInstance() {
        if (instance == null) instance = new LEDSubsystem();
        return instance;
    }
}