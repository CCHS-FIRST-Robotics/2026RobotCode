package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    class ClimberIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;
    }
    
    public default void updateInputs(ClimberIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}
}