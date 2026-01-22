package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    protected final StatusSignal<Voltage> voltageSignal;
    protected final StatusSignal<Current> currentSignal;
    protected final StatusSignal<Angle> positionSignal;
    protected final StatusSignal<AngularVelocity> velocitySignal;
    protected final StatusSignal<Temperature> temperatureSignal;

    public ShooterIOReal(int id) {
        motor = new TalonFX(id);

        // motor config
        motorConfig.Slot0 = FuelConstants.SHOOTER_PID;
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
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            voltageSignal, 
            currentSignal, 
            positionSignal, 
            velocitySignal, 
            temperatureSignal
        );
        
        inputs.shooterVoltage = voltageSignal.getValue().in(Volts);
        inputs.shooterCurrent = currentSignal.getValue().in(Amps);
        inputs.shooterPosition = positionSignal.getValue().in(Rotations);
        inputs.shooterVelocity = velocitySignal.getValue().in(RotationsPerSecond);
        inputs.shooterTemperature = temperatureSignal.getValue().in(Celsius);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityVoltageRequest.withVelocity(velocity));
    }
}