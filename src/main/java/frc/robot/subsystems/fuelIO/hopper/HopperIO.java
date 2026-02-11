package frc.robot.subsystems.fuelIO.hopper;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    class HopperIOInputs {
        public double hopperVoltage;
        public double hopperCurrent;
        public double hopperPosition;
        public double hopperVelocity;
        public double hopperTemperature;
    }
    
    public default void updateInputs(HopperIOInputs inputs) {}
    
    public default void setHopperVoltage(Voltage volts) {}
}