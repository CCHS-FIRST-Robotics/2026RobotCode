package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}
}