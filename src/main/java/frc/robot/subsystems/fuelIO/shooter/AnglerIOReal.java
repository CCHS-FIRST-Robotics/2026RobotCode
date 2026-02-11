package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class AnglerIOReal implements AnglerIO {
    private final SparkMax anglerMotor;
    private final SparkMaxConfig anglerConfig = new SparkMaxConfig();
    private final RelativeEncoder anglerEncoder;

    public AnglerIOReal(int anglerId) {
        anglerMotor = new SparkMax(anglerId, MotorType.kBrushless);

        // start config
        anglerMotor.setCANTimeout(500);

        // encoders
        anglerEncoder = anglerMotor.getEncoder();
        anglerEncoder.setPosition(0.0); // ! 

        // pid 
        anglerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.ANGLER_PID);
        anglerConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(0.1).in(Rotations.per(Minute))); // ! 
        anglerConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(100).in(Rotations.per(Minute).per(Second)));
        anglerConfig.closedLoop.maxMotion.allowedProfileError(Rotations.of(0.05).in(Rotations));

        // miscellaneous settings
        anglerConfig.signals.primaryEncoderVelocityPeriodMs(10);
        anglerConfig.encoder.quadratureMeasurementPeriod(10);
        anglerConfig.encoder.quadratureAverageDepth(2);

        anglerConfig.smartCurrentLimit(30);
        anglerConfig.voltageCompensation(12);

        // ! inverted

        anglerConfig.idleMode(IdleMode.kCoast);

        anglerConfig.encoder
            .positionConversionFactor(1 / FuelConstants.ANGLER_GEAR_RATIO)
            .velocityConversionFactor(1 / FuelConstants.ANGLER_GEAR_RATIO);

        // stop config
        anglerMotor.setCANTimeout(0);
        anglerMotor.configure(anglerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AnglerIOInputs inputs) {
        inputs.anglerVoltage = anglerMotor.getAppliedOutput() * anglerMotor.getBusVoltage();
        inputs.anglerCurrent = anglerMotor.getOutputCurrent();
        inputs.anglerPosition = anglerEncoder.getPosition();
        inputs.anglerVelocity = Rotations.per(Minute).of(anglerEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.anglerTemperature = anglerMotor.getMotorTemperature();
    }

    @Override
    public void setAnglerVoltage(Voltage volts) {
        anglerMotor.setVoltage(volts);
    }

    @Override
    public void setAnglerPosition(Angle angle) {
        anglerMotor.getClosedLoopController().setSetpoint(
            angle.in(Rotations), 
            SparkMax.ControlType.kMAXMotionPositionControl, 
            ClosedLoopSlot.kSlot0,
            FuelConstants.ANGLER_KCOS * Math.cos(Rotations.of(anglerEncoder.getPosition()).in(Radians))
        );
    }
}