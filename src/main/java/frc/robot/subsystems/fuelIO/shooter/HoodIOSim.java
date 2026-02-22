package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HoodIOSim implements HoodIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.00001, 
            1
        ), 
        DCMotor.getKrakenX60(1)
    );

    private final PIDController PID = new PIDController(150, 0, 0);

    private Voltage appliedVoltage = Volts.of(0);

    public HoodIOSim() {
        motor.setState(Constants.HOOD_START_ANGLE.in(Radians) * FuelConstants.HOOD_GEAR_RATIO, 0);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.voltage = appliedVoltage.in(Volts);
        inputs.current = motor.getCurrentDrawAmps();
        inputs.position = motor.getAngularPositionRotations() / FuelConstants.HOOD_GEAR_RATIO;
        inputs.velocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.HOOD_GEAR_RATIO;
        inputs.temperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(Angle angle) {
        double volts = PID.calculate(
            motor.getAngularPositionRotations() / FuelConstants.HOOD_GEAR_RATIO, 
            angle.in(Rotations)
        );
        motor.setInputVoltage(volts);
        
        appliedVoltage = Volts.of(volts);
    }
}