package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    private final RelativeEncoder intakeEncoder;

    public IntakeIOReal(int id) {
        intakeMotor = new SparkMax(id, MotorType.kBrushless);

        // start config
        intakeMotor.setCANTimeout(500);

        // encoders
        intakeEncoder = intakeMotor.getEncoder();
        intakeEncoder.setPosition(0.0);

        // miscellaneous settings
        intakeConfig.signals.primaryEncoderVelocityPeriodMs(10);
        intakeConfig.encoder.quadratureMeasurementPeriod(10);
        intakeConfig.encoder.quadratureAverageDepth(2);

        intakeConfig.smartCurrentLimit(30);
        intakeConfig.voltageCompensation(12);

        intakeConfig.idleMode(IdleMode.kCoast);

        intakeConfig.encoder
            .positionConversionFactor(1 / FuelConstants.INTAKE_GEAR_RATIO)
            .velocityConversionFactor(1 / FuelConstants.INTAKE_GEAR_RATIO);

        // stop config
        intakeMotor.setCANTimeout(0);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeCurrent = intakeMotor.getOutputCurrent();
        inputs.intakePosition = intakeEncoder.getPosition();
        inputs.intakeVelocity = Rotations.per(Minute).of(intakeEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.intakeTemperature = intakeMotor.getMotorTemperature();
    }

    @Override
    public void setIntakeVoltage(Voltage volts) {
        intakeMotor.setVoltage(volts);
    }
}