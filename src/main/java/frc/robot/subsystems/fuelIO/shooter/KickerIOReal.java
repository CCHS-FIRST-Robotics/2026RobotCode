package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class KickerIOReal implements KickerIO {
    private final SparkMax kickerMotor;
    private final SparkMaxConfig kickerConfig = new SparkMaxConfig();
    private final RelativeEncoder kickerEncoder;

    public KickerIOReal(int kickerId) {
        kickerMotor = new SparkMax(kickerId, MotorType.kBrushless);

        // start config
        kickerMotor.setCANTimeout(500);

        // encoders
        kickerEncoder = kickerMotor.getEncoder();
        kickerEncoder.setPosition(0.0);

        // miscellaneous settings
        kickerConfig.signals.primaryEncoderVelocityPeriodMs(10);
        kickerConfig.encoder.quadratureMeasurementPeriod(10);
        kickerConfig.encoder.quadratureAverageDepth(2);

        kickerConfig.smartCurrentLimit(30);
        kickerConfig.voltageCompensation(12);

        // ! inverted

        kickerConfig.idleMode(IdleMode.kCoast);

        kickerConfig.encoder
            .positionConversionFactor(1 / FuelConstants.KICKER_GEAR_RATIO)
            .velocityConversionFactor(1 / FuelConstants.KICKER_GEAR_RATIO);

        // stop config
        kickerMotor.setCANTimeout(0);
        kickerMotor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.kickerVoltage = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        inputs.kickerCurrent = kickerMotor.getOutputCurrent();
        inputs.kickerPosition = kickerEncoder.getPosition();
        inputs.kickerVelocity = Rotations.per(Minute).of(kickerEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.kickerTemperature = kickerMotor.getMotorTemperature();
    }

    @Override
    public void setKickerVoltage(Voltage volts) {
        kickerMotor.setVoltage(volts);
    }
}