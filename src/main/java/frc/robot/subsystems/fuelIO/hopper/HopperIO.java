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
    
    public default void setIntakeVoltage(Voltage volts) {}

    public default void setIntakeVelocity(AngularVelocity velocity) {}

    public default void setPivotVoltage(Voltage volts) {}

    public default void setPivotPosition(Angle angle) {}
}