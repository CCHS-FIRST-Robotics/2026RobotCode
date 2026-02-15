package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface AnglerIO {
    @AutoLog
    class AnglerIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;
    }

    public default void updateInputs(AnglerIOInputs inputs) {}

    public default void setVoltage(Voltage volts) {}

    public default void setPosition(Angle angle) {}
}