package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.*;

public class LedStrip extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private int rainbowHue = 0;

    public LedStrip() {
        led = new AddressableLED(LedStripConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LedStripConstants.BULB_COUNT);
        
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
    
    @Override
    public void periodic() {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, rainbowHue, 255, 192);
        }
        
        rainbowHue = (rainbowHue + 1) % 180;
        led.setData(ledBuffer);
    }
}
