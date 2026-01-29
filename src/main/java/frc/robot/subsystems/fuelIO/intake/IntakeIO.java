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

        public double pivotVoltage;
        public double pivotCurrent;
        public double pivotPosition;
        public double pivotVelocity;
        public double pivotTemperature;
    }
    
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setIntakeVoltage(Voltage volts) {}

    public default void setIntakeVelocity(AngularVelocity velocity) {}

    public default void setPivotVoltage(Voltage volts) {}

    public default void setPivotPosition(Angle angle) {}
}