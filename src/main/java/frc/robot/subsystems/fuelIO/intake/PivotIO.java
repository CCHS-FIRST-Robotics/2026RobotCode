package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;

        public double positionSetpoint;
    }
    
    public default void updateInputs(PivotIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}

    public default void setPosition(Angle angle) {}

    public default void setEncoderPosition(Angle angle) {}
}