package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    // ! gearing and JKgMetersSquared
    private final DCMotorSim intakeMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), 
        DCMotor.getNEO(1)
    );

    private final DCMotorSim pivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1), 
        DCMotor.getNEO(1)
    );

    private final PIDController pivotPID = new PIDController(15, 0, 0);

    private Voltage appliedIntakeVoltage = Volts.of(0);
    private Voltage appliedPivotVoltage = Volts.of(0);

    public IntakeIOSim() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeMotor.update(Constants.PERIOD);
        pivotMotor.update(Constants.PERIOD);

        inputs.intakeVoltage = appliedIntakeVoltage.in(Volts);
        inputs.intakeCurrent = intakeMotor.getCurrentDrawAmps();
        inputs.intakePosition = intakeMotor.getAngularPositionRotations();
        inputs.intakeVelocity = Rotations.per(Minute).of(intakeMotor.getAngularVelocityRPM()).in(RotationsPerSecond);
        inputs.intakeTemperature = Celsius.of(20).in(Celsius);

        inputs.pivotVoltage = appliedPivotVoltage.in(Volts);
        inputs.pivotCurrent = intakeMotor.getCurrentDrawAmps();
        inputs.pivotPosition = intakeMotor.getAngularPositionRotations();
        inputs.pivotVelocity = Rotations.per(Minute).of(intakeMotor.getAngularVelocityRPM()).in(RotationsPerSecond);
        inputs.pivotTemperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setIntakeVoltage(Voltage volts) {
        intakeMotor.setInputVoltage(volts.in(Volts));
        appliedIntakeVoltage = volts;
    }

    @Override
    public void setPivotVoltage(Voltage volts) {
        pivotMotor.setInputVoltage(volts.in(Volts));
        appliedPivotVoltage = volts;
    }

    @Override
    public void setPivotPosition(Angle angle) {
        double volts = pivotPID.calculate(angle.in(Rotations));
        pivotMotor.setInputVoltage(volts);
        appliedPivotVoltage = Volts.of(volts);
    }
}