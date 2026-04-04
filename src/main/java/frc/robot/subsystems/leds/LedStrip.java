package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;


public class LedStrip extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private Integer[] hues = new Integer[0];

    private int rainbowFirstPixelHue = 0;
    private int frequency = 2;
    
    public LedStrip() {
        led = new AddressableLED(LedStripConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LedStripConstants.BULB_COUNT);
        
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        if (hues.length == 0) {
            for (int i = 0; i < ledBuffer.getLength(); i++) {
                int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
                ledBuffer.setHSV(i, hue, 255, 128);
            }
            
            led.setData(ledBuffer);
            
            Logger.recordOutput("outputs/ledStrip/color", Color.fromHSV(rainbowFirstPixelHue, 255, 255));

            rainbowFirstPixelHue = (rainbowFirstPixelHue + 3) % 180;
            return;
        }
        
        int hue = hues[(int) Timer.getFPGATimestamp() * frequency % hues.length];

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, 255, 255);
        }
        led.setData(ledBuffer);

        Logger.recordOutput("outputs/ledStrip/color", Color.fromHSV(hue, 255, 255));
    }

    public void setLedStripHues(Integer[] hues) {
        this.hues = hues;
    }

    public Command getSetLedStripHuesCommand(Integer[] hues) {
        return runOnce(() -> setLedStripHues(hues));
    }
}
