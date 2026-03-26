package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.*;

public class LedStrip extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;
    private final AddressableLEDBufferView leftBuffer;
    private final AddressableLEDBufferView rightBuffer;

    private int rainbowHue = 0;

    public LedStrip() {
        led = new AddressableLED(LedStripConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LedStripConstants.BULB_COUNT);
        leftBuffer = ledBuffer.createView(0, 59);
        rightBuffer = ledBuffer.createView(60, 119);
        
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
    
    @Override
    public void periodic() {
        for (var i = 0; i < ledBuffer.getLength() / 2; i++) {
            leftBuffer.setHSV(i, rainbowHue, 255, 255);
            rightBuffer.setHSV(i, rainbowHue, 255, 255);
        }
        
        rainbowHue = (rainbowHue + 3) % 180;
        led.setData(ledBuffer);
    }
}
