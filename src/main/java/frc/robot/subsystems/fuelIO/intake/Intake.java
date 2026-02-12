package frc.robot.subsystems.fuelIO.intake;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeIOInputs = new IntakeIOInputsAutoLogged();

    private final PivotIO pivotIO;
    private final PivotIOInputsAutoLogged pivotIOInputs = new PivotIOInputsAutoLogged();

    Angle pivotAngle = FuelConstants.PIVOT_START_ANGLE;

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
        Logger.processInputs("intake", intakeIOInputs);
        pivotIO.updateInputs(pivotIOInputs);
        Logger.processInputs("pivot", pivotIOInputs);

        if(Constants.ENABLE_PIVOT_SET_POSITION){
            pivotIO.setPosition(pivotAngle);
        }
    }

    // ————— raw command factories ————— //

    public Command getSetIntakeVoltageCommand(Voltage volts) {
        return runOnce(() -> intakeIO.setVoltage(volts));
    }

    // ! do not call if pivotIO.setPosition() is active
    public Command getSetPivotVoltageCommand(Voltage volts) {
        return runOnce(() -> pivotIO.setVoltage(volts));
    }

    public Command getSetPivotPositionCommand(Angle angle) {
        return runOnce(() -> pivotAngle = angle);
    }

    public boolean getIntakeOn() {
        return Math.abs(intakeIOInputs.intakeVoltage) > 0;
    }

    // ————— processed command factories ————— //
}