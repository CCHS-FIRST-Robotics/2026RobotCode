package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;


public class LedStrip extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer ledBuffer;

    private Integer[] hues;
    
    public LedStrip() {
        led = new AddressableLED(LedStripConstants.PWM_PORT);
        ledBuffer = new AddressableLEDBuffer(LedStripConstants.BULB_COUNT);
        
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }

    @Override
    public void periodic() {
        int hue = hues[(int) Timer.getFPGATimestamp() % hues.length];

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, 255, 255);
        }
        led.setData(ledBuffer);

        Logger.recordOutput("outputs/ledStrip/color", Color.fromHSV(hue, 255, 255));
    }

    public void setLedStripHues(Integer[] hues) {
        this.hues = hues;
    }

    public Command getSetLedStripHueCommand(Integer[] hues) {
        return runOnce(() -> setLedStripHues(hues));
    }
}
