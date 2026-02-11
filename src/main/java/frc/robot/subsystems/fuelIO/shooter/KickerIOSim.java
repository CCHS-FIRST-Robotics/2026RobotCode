package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class KickerIOSim implements KickerIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.00001, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private Voltage appliedVoltage = Volts.of(0);

    public KickerIOSim() {

    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.kickerVoltage = appliedVoltage.in(Volts);
        inputs.kickerCurrent = motor.getCurrentDrawAmps();
        inputs.kickerPosition = motor.getAngularPositionRotations() / FuelConstants.INTAKE_GEAR_RATIO;
        inputs.kickerVelocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.INTAKE_GEAR_RATIO;
        inputs.kickerTemperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }
}