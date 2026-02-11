package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class PivotIOSim implements PivotIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.00001, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private final PIDController PID = new PIDController(15, 0, 0);

    private Voltage appliedVoltage = Volts.of(0);

    public PivotIOSim() {

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.pivotVoltage = appliedVoltage.in(Volts);
        inputs.pivotCurrent = motor.getCurrentDrawAmps();
        inputs.pivotPosition = motor.getAngularPositionRotations() / FuelConstants.PIVOT_GEAR_RATIO;
        inputs.pivotVelocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.PIVOT_GEAR_RATIO;
        inputs.pivotTemperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }

    @Override
    public void setPosition(Angle angle) {
        double volts = PID.calculate(
            motor.getAngularPositionRotations() / FuelConstants.PIVOT_GEAR_RATIO, 
            angle.in(Rotations)
        );
        motor.setInputVoltage(volts);
        
        appliedVoltage = Volts.of(volts);
    }
}