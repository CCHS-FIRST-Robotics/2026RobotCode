package frc.robot.subsystems.fuelIO.hopper;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HopperIOReal implements HopperIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    public HopperIOReal(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);

        // start config
        motor.setCANTimeout(500);

        // encoders
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        // miscellaneous settings
        motorConfig.signals.primaryEncoderVelocityPeriodMs(10);
        motorConfig.encoder.quadratureMeasurementPeriod(10);
        motorConfig.encoder.quadratureAverageDepth(2);

        motorConfig.smartCurrentLimit(30);
        motorConfig.voltageCompensation(12);

        motorConfig.idleMode(IdleMode.kCoast);

        motorConfig.encoder
        .positionConversionFactor(1 / FuelConstants.HOPPER_GEAR_RATIO)
        .velocityConversionFactor(1 / FuelConstants.HOPPER_GEAR_RATIO);

        // stop config
        motor.setCANTimeout(0);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.hopperCurrent = motor.getOutputCurrent();
        inputs.hopperPosition = encoder.getPosition();
        inputs.hopperVelocity = Rotations.per(Minute).of(encoder.getVelocity()).in(RotationsPerSecond);
        inputs.hopperTemperature = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }
}