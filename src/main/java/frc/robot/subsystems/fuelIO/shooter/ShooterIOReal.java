package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX motor;
    private final TalonFX follower;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0);

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public AngularVelocity velocitySetpoint = RotationsPerSecond.of(0);

    public ShooterIOReal(int motorId, int followerId) {
        motor = new TalonFX(motorId);
        follower = new TalonFX(followerId);

        // motor config
        motorConfig.Slot0 = FuelConstants.SHOOTER_PIDF;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(motorConfig);

        follower.getConfigurator().apply(motorConfig);
        follower.setControl(new Follower(motorId, MotorAlignmentValue.Opposed));

        // status signals
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getStatorCurrent();
        positionSignal = motor.getPosition();
        velocitySignal = motor.getVelocity();
        temperatureSignal = motor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            voltageSignal, 
            currentSignal, 
            positionSignal, 
            velocitySignal, 
            temperatureSignal
        );
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            voltageSignal, 
            currentSignal, 
            positionSignal, 
            velocitySignal, 
            temperatureSignal
        );
        
        inputs.voltage = voltageSignal.getValue().in(Volts);
        inputs.current = currentSignal.getValue().in(Amps);
        inputs.position = positionSignal.getValue().in(Rotations);
        inputs.velocity = velocitySignal.getValue().in(RotationsPerSecond);
        inputs.temperature = temperatureSignal.getValue().in(Celsius);

        inputs.velocitySetpoint = velocitySetpoint.in(RotationsPerSecond);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltageRequest.withVelocity(velocity));

        velocitySetpoint = velocity;
    }
}