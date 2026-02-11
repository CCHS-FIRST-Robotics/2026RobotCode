package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    class KickerIOInputs {
        public double kickerVoltage;
        public double kickerCurrent;
        public double kickerPosition;
        public double kickerVelocity;
        public double kickerTemperature;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setKickerVoltage(Voltage volts) {}
}