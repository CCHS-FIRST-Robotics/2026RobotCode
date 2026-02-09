package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.fuelIO.FuelConstants;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    Angle pivotAngle = FuelConstants.INTAKE_START_ANGLE;

    boolean autoVelocity;

    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);

        io.setPivotPosition(pivotAngle);
    }

    // ————— raw command factories ————— //

    public Command getSetIntakeVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setIntakeVoltage(volts));
    }

    public Command getSetIntakeVelocityCommand(AngularVelocity velocity) { // ! idk if this is necessary
        return runOnce(() -> io.setIntakeVelocity(velocity));
    }

    public Command getSetPivotVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setPivotVoltage(volts));
    }

    public Command getSetPivotPositionCommand(Angle angle) {
        return runOnce(() -> pivotAngle = angle);
    }

    // ————— processed command factories ————— //
}