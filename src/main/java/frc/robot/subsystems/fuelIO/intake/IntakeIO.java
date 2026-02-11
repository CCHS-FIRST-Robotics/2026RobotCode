package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double intakeVoltage;
        public double intakeCurrent;
        public double intakePosition;
        public double intakeVelocity;
        public double intakeTemperature;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setVoltage(Voltage volts) {}
}