package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AnglerIO {
    @AutoLog
    class AnglerIOInputs {
        public double anglerVoltage;
        public double anglerCurrent;
        public double anglerPosition;
        public double anglerVelocity;
        public double anglerTemperature;
    }

    public default void updateInputs(AnglerIOInputs inputs) {}

    public default void setAnglerVoltage(Voltage volts) {}

    public default void setAnglerPosition(Angle angle) {}
}