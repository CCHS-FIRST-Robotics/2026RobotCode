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
    private final SparkMax hopperMotor;
    private final SparkMaxConfig hopperConfig = new SparkMaxConfig();
    private final RelativeEncoder hopperEncoder;

    public HopperIOReal(int id) {
        hopperMotor = new SparkMax(id, MotorType.kBrushless);

        // start config
        hopperMotor.setCANTimeout(500);

        // encoders
        hopperEncoder = hopperMotor.getEncoder();
        hopperEncoder.setPosition(0.0);

        // miscellaneous settings
        hopperConfig.signals.primaryEncoderVelocityPeriodMs(10);
        hopperConfig.encoder.quadratureMeasurementPeriod(10);
        hopperConfig.encoder.quadratureAverageDepth(2);

        hopperConfig.smartCurrentLimit(30);
        hopperConfig.voltageCompensation(12);

        hopperConfig.idleMode(IdleMode.kCoast);

        hopperConfig.encoder
            .positionConversionFactor(1 / FuelConstants.HOPPER_GEAR_RATIO)
            .velocityConversionFactor(1 / FuelConstants.HOPPER_GEAR_RATIO);

        // stop config
        hopperMotor.setCANTimeout(0);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperVoltage = hopperMotor.getAppliedOutput() * hopperMotor.getBusVoltage();
        inputs.hopperCurrent = hopperMotor.getOutputCurrent();
        inputs.hopperPosition = hopperEncoder.getPosition();
        inputs.hopperVelocity = Rotations.per(Minute).of(hopperEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.hopperTemperature = hopperMotor.getMotorTemperature();
    }

    @Override
    public void setHopperVoltage(Voltage volts) {
        hopperMotor.setVoltage(volts);
    }
}