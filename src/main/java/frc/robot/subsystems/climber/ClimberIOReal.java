package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    public ClimberIOReal(int id) {
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

        motorConfig.inverted(true);

        motorConfig.idleMode(IdleMode.kCoast);

        motorConfig.encoder
        .positionConversionFactor(1 / FuelConstants.INTAKE_GEAR_RATIO)
        .velocityConversionFactor(1 / FuelConstants.INTAKE_GEAR_RATIO);

        // stop config
        motor.setCANTimeout(0);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.current = motor.getOutputCurrent();
        inputs.position = encoder.getPosition();
        inputs.velocity = Rotations.per(Minute).of(encoder.getVelocity()).in(RotationsPerSecond);
        inputs.temperature = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }
}