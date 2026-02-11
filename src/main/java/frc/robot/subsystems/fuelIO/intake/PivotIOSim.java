package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class PivotIOSim implements PivotIO {
    // ! gearing and JKgMetersSquared
    private final DCMotorSim pivotMotor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.00001, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private final PIDController pivotPID = new PIDController(15, 0, 0);

    private Voltage appliedPivotVoltage = Volts.of(0);

    public PivotIOSim() {

    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        pivotMotor.update(Constants.PERIOD);

        inputs.pivotVoltage = appliedPivotVoltage.in(Volts);
        inputs.pivotCurrent = pivotMotor.getCurrentDrawAmps();
        inputs.pivotPosition = pivotMotor.getAngularPositionRotations() / FuelConstants.PIVOT_GEAR_RATIO;
        inputs.pivotVelocity = Rotations.per(Minute).of(pivotMotor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.PIVOT_GEAR_RATIO;
        inputs.pivotTemperature = Celsius.of(20).in(Celsius);
    }

    @Override
    public void setPivotVoltage(Voltage volts) {
        pivotMotor.setInputVoltage(volts.in(Volts));
        
        appliedPivotVoltage = volts;
    }

    @Override
    public void setPivotPosition(Angle angle) {
        double volts = pivotPID.calculate(
            pivotMotor.getAngularPositionRotations() / FuelConstants.PIVOT_GEAR_RATIO, 
            angle.in(Rotations)
        );
        pivotMotor.setInputVoltage(volts);
        
        appliedPivotVoltage = Volts.of(volts);
    }
}