package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ShooterIOReal implements ShooterIO {
    private final TalonFX shooterMotor;
    private final TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

    private final VoltageOut shooterVoltageRequest = new VoltageOut(0);
    private final VelocityVoltage shooterVelocityVoltageRequest = new VelocityVoltage(0.0);

    private final StatusSignal<Voltage> shooterVoltageSignal;
    private final StatusSignal<Current> shooterCurrentSignal;
    private final StatusSignal<Angle> shooterPositionSignal;
    private final StatusSignal<AngularVelocity> shooterVelocitySignal;
    private final StatusSignal<Temperature> shooterTemperatureSignal;

    public ShooterIOReal(int shooterId) {
        shooterMotor = new TalonFX(shooterId);

        // motor config
        shooterConfig.Slot0 = FuelConstants.SHOOTER_PIDF;
        shooterConfig.CurrentLimits.StatorCurrentLimit = 40;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        shooterMotor.getConfigurator().apply(shooterConfig);

        // status signals
        shooterVoltageSignal = shooterMotor.getMotorVoltage();
        shooterCurrentSignal = shooterMotor.getStatorCurrent();
        shooterPositionSignal = shooterMotor.getPosition();
        shooterVelocitySignal = shooterMotor.getVelocity();
        shooterTemperatureSignal = shooterMotor.getDeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            shooterVoltageSignal, 
            shooterCurrentSignal, 
            shooterPositionSignal, 
            shooterVelocitySignal, 
            shooterTemperatureSignal
        );
        ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            shooterVoltageSignal, 
            shooterCurrentSignal, 
            shooterPositionSignal, 
            shooterVelocitySignal, 
            shooterTemperatureSignal
        );
        
        inputs.shooterVoltage = shooterVoltageSignal.getValue().in(Volts);
        inputs.shooterCurrent = shooterCurrentSignal.getValue().in(Amps);
        inputs.shooterPosition = shooterPositionSignal.getValue().in(Rotations);
        inputs.shooterVelocity = shooterVelocitySignal.getValue().in(RotationsPerSecond);
        inputs.shooterTemperature = shooterTemperatureSignal.getValue().in(Celsius);
    }

    @Override
    public void setShooterVoltage(Voltage volts) {
        shooterMotor.setControl(shooterVoltageRequest.withOutput(volts));
    }

    @Override
    public void setShooterVelocity(AngularVelocity velocity) {
        shooterMotor.setControl(shooterVelocityVoltageRequest.withVelocity(velocity));
    }
}