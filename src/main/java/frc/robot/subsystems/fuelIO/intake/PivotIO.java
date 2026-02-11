package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public double pivotVoltage;
        public double pivotCurrent;
        public double pivotPosition;
        public double pivotVelocity;
        public double pivotTemperature;
    }
    
    public default void updateInputs(PivotIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}

    public default void setPosition(Angle angle) {}
}