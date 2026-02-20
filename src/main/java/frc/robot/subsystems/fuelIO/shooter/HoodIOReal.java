package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HoodIOReal implements HoodIO {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;
    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Temperature> temperatureSignal;

    public HoodIOReal(int id) {
        motor = new TalonFX(id);

        // motor config
        motorConfig.Slot0 = FuelConstants.HOOD_PIDF;
        motorConfig.CurrentLimits.StatorCurrentLimit = 40;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(motorConfig);

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
    public void updateInputs(HoodIOInputs inputs) {
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
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setPosition(Angle angle) {
        motor.setControl(positionVoltageRequest.withPosition(angle));
    }
}