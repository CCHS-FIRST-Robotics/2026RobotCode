package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;

        public double velocitySetpoint;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}

    public default void setVelocity(AngularVelocity velocity) {}
}