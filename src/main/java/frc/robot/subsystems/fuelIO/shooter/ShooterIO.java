package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs {
        public double shooterVoltage;
        public double shooterCurrent;
        public double shooterPosition;
        public double shooterVelocity;
        public double shooterTemperature;
    }
    
    public default void setVoltage(Voltage volts) {}

    public default void setVelocity(AngularVelocity velocityRadPerSec) {}

    public default void updateInputs(ShooterIOInputs inputs) {}
}