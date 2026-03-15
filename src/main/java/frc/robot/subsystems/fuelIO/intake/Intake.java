package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    public final IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

    private final PivotIO pivotIO;
    public final PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    Angle pivotAngle = Constants.PIVOT_START_ANGLE;

    public Intake(
        IntakeIO intakeIO, 
        PivotIO pivotIO
    ) {
        this.intakeIO = intakeIO;
        this.pivotIO = pivotIO;
    }
    
    @Override
    public void periodic() {
        intakeIO.updateInputs(intakeIOInputs);
        Logger.processInputs("subsystems/fuelIO/intake/intake", intakeIOInputs);
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("subsystems/fuelIO/intake/pivot", pivotIOInputs);

        if (Constants.USE_PIVOT) {
            pivotIO.setPosition(pivotAngle); // NEO PID and sim PIDs must be called externally
        }
    }

    // ————— raw command factories ————— //

    // intake

    public void setIntakeVoltage(Voltage volts) {
        if (pivotDown()) {
            intakeIO.setVoltage(volts);
        }
    }

    public Command getSetIntakeVoltageCommand(Voltage volts) {
        return runOnce(() -> setIntakeVoltage(volts));
    }

    // pivot

    public void setPivotVoltage(Voltage volts) {
        pivotIO.setVoltage(volts);
    }

    public Command getSetPivotVoltageCommand(Voltage volts) {
        return runOnce(() -> setPivotVoltage(volts));
    }

    public void setPivotPosition(Angle angle) {
        pivotAngle = angle;
    }

    public Command getSetPivotPositionCommand(Angle angle) {
        return runOnce(() -> setPivotPosition(angle));
    }

    // util

    public boolean getIntakeOn() {
        return Math.abs(intakeIOInputs.voltage) > 0;
    }

    public boolean pivotDown() {
        return pivotIOInputs.position < 0;
    }
}