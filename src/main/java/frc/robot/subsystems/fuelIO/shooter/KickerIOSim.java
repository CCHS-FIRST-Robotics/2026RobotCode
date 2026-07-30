/**
 * Original code
 */

package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.*;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class KickerIOSim implements KickerIO {
    private final DCMotorSim motor = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNEO(1), 
            0.01, 
            1
        ), 
        DCMotor.getNEO(1)
    );

    private final PIDController PID = new PIDController(2, 0, 0);
    private final SimpleMotorFeedforward FF = new SimpleMotorFeedforward(0, 0.1209, 0);

    private Voltage appliedVoltage = Volts.of(0);
    private AngularVelocity velocitySetpoint = RotationsPerSecond.of(0);

    public KickerIOSim() {
        motor.setState(0, 0);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        motor.update(Constants.PERIOD);

        inputs.voltage = appliedVoltage.in(Volts);
        inputs.current = motor.getCurrentDrawAmps();
        inputs.position = motor.getAngularPositionRotations() / FuelConstants.SHOOTER_GEAR_RATIO;
        inputs.velocity = Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.SHOOTER_GEAR_RATIO;
        inputs.temperature = Celsius.of(20).in(Celsius);

        inputs.velocitySetpoint = velocitySetpoint.in(RotationsPerSecond);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setInputVoltage(volts.in(Volts));
        
        appliedVoltage = volts;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        double volts = PID.calculate(
            Rotations.per(Minute).of(motor.getAngularVelocityRPM()).in(RotationsPerSecond) / FuelConstants.SHOOTER_GEAR_RATIO, 
            velocity.in(RotationsPerSecond)
        ) + FF.calculateWithVelocities(
            velocitySetpoint.in(RotationsPerSecond),
            velocity.in(RotationsPerSecond)
        );
        motor.setInputVoltage(volts);
        
        appliedVoltage = Volts.of(volts);
        velocitySetpoint = velocity;
    }
}