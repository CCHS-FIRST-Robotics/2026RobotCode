/**
 * Original code
 */

package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    class KickerIOInputs {
        public double voltage;
        public double current;
        public double position;
        public double velocity;
        public double temperature;

        public double velocitySetpoint;
    }

    public default void updateInputs(KickerIOInputs inputs) {}

    public default void setVoltage(Voltage volts) {}

    public default void setVelocity(AngularVelocity velocity) {}
}