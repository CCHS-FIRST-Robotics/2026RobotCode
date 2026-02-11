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
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    public AnglerIOReal(int anglerId) {
        motor = new SparkMax(anglerId, MotorType.kBrushless);

        // start config
        motor.setCANTimeout(500);

        // encoders
        encoder = motor.getEncoder();
        encoder.setPosition(0.0); // ! 

        // pid 
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.ANGLER_PID);
        motorConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(0.1).in(Rotations.per(Minute))); // ! 
        motorConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(100).in(Rotations.per(Minute).per(Second)));
        motorConfig.closedLoop.maxMotion.allowedProfileError(Rotations.of(0.05).in(Rotations));

        // miscellaneous settings
        motorConfig.signals.primaryEncoderVelocityPeriodMs(10);
        motorConfig.encoder.quadratureMeasurementPeriod(10);
        motorConfig.encoder.quadratureAverageDepth(2);

        motorConfig.smartCurrentLimit(30);
        motorConfig.voltageCompensation(12);

        // ! inverted

        motorConfig.idleMode(IdleMode.kCoast);

        motorConfig.encoder
        .positionConversionFactor(1 / FuelConstants.ANGLER_GEAR_RATIO)
        .velocityConversionFactor(1 / FuelConstants.ANGLER_GEAR_RATIO);

        // stop config
        motor.setCANTimeout(0);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(AnglerIOInputs inputs) {
        inputs.anglerVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.anglerCurrent = motor.getOutputCurrent();
        inputs.anglerPosition = encoder.getPosition();
        inputs.anglerVelocity = Rotations.per(Minute).of(encoder.getVelocity()).in(RotationsPerSecond);
        inputs.anglerTemperature = motor.getMotorTemperature();
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(Angle angle) {
        motor.getClosedLoopController().setSetpoint(
            angle.in(Rotations), 
            SparkMax.ControlType.kMAXMotionPositionControl, 
            ClosedLoopSlot.kSlot0, 
            FuelConstants.ANGLER_KCOS * Math.cos(Rotations.of(encoder.getPosition()).in(Radians))
        );
    }
}