/**
 * Original code
 */

package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    public final IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

    private final PivotIO pivotIO;
    public final PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    Angle pivotAngle = Constants.PIVOT_START_ANGLE;

    int hopperFuel;

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

        if (Constants.ENABLE_PIVOT) {
            pivotIO.setPosition(pivotAngle); // NEO and sim PIDs must be called externally
        } else {
            pivotIO.setVoltage(Volts.of(0));
        }
    }

    // ————— raw command factories ————— //

    // intake

    public void setIntakeVoltage(Voltage volts, boolean pivotDownOverride) {
        if (pivotDown() || pivotDownOverride) {
            intakeIO.setVoltage(volts);
        }
    }

    public Command getSetIntakeVoltageCommand(Voltage volts, boolean pivotDownOverride) {
        return runOnce(() -> setIntakeVoltage(volts, pivotDownOverride));
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

    public void setPivotEncoderPositionUp() {
        pivotIO.setEncoderPosition(FuelConstants.PIVOT_MAX_UP_ANGLE);
        pivotAngle = FuelConstants.PIVOT_MAX_UP_ANGLE;
    }

    public void setPivotEncoderPositionDown() {
        pivotIO.setEncoderPosition(FuelConstants.PIVOT_MAX_DOWN_ANGLE);
        pivotAngle = FuelConstants.PIVOT_MAX_DOWN_ANGLE;
    }

    // util

    public boolean getIntakeOn() {
        return Math.abs(intakeIOInputs.voltage) > 0;
    }

    @AutoLogOutput(key = "outputs/fuelIO/intake/pivotDown")
    public boolean pivotDown() {
        return pivotIOInputs.position < FuelConstants.PIVOT_MAX_DOWN_ANGLE.in(Rotations) + 0.05;
    }

    public void addHopperFuel() {
        hopperFuel++;
    }

    public void subtractHopperFuel() {
        hopperFuel--;
    }

    @AutoLogOutput(key = "outputs/fuelIO/intake/hopperFuel")
    public int getHopperFuel() {
        return hopperFuel;
    }

    public boolean getHopperFull() {
        return hopperFuel >= FuelConstants.HOPPER_FUEL_CAPACITY;
    }

    public boolean getHopperEmpty() {
        return hopperFuel <= 0;
    }
}