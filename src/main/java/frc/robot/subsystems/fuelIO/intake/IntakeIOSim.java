package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.00001, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private Voltage appliedVoltage = Volts.of(0);

    public IntakeIOSim() {

    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.intakeVoltage = appliedVoltage.in(Volts);
        inputs.intakeCurrent = motor.getCurrentDrawAmps();
        inputs.intakePosition = motor.getAngularPositionRotations() / FuelConstants.INTAKE_GEAR_RATIO;
        inputs.intakeVelocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.INTAKE_GEAR_RATIO;
        inputs.intakeTemperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }
}