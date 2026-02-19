package frc.robot.subsystems.fuelIO.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HopperIOSim implements HopperIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.00001, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private Voltage appliedVoltage = Volts.of(0);

    public HopperIOSim() {
        motor.setState(0, 0);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.voltage = appliedVoltage.in(Volts);
        inputs.current = motor.getCurrentDrawAmps();
        inputs.position = motor.getAngularPositionRotations() / FuelConstants.HOPPER_GEAR_RATIO;
        inputs.velocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.HOPPER_GEAR_RATIO;
        inputs.temperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }
}