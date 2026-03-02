package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class PivotIOReal implements PivotIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;

    private Angle positionSetpoint = Rotations.of(0);

    public PivotIOReal(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);

        // start config
        motor.setCANTimeout(500);

        // encoders
        encoder = motor.getEncoder();
        encoder.setPosition(Constants.PIVOT_START_ANGLE.in(Rotations));

        // pid 
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.PIVOT_PID);
        motorConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(0.25).in(Rotations.per(Minute)));
        motorConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(100).in(Rotations.per(Minute).per(Second)));
        motorConfig.closedLoop.maxMotion.allowedProfileError(Rotations.of(0.05).in(Rotations));

        // miscellaneous settings
        motorConfig.signals.primaryEncoderVelocityPeriodMs(10);
        motorConfig.encoder.quadratureMeasurementPeriod(10);
        motorConfig.encoder.quadratureAverageDepth(2);
        
        motorConfig.smartCurrentLimit(30);
        motorConfig.voltageCompensation(12);
        
        motorConfig.inverted(true);
        
        motorConfig.idleMode(IdleMode.kCoast);

        motorConfig.encoder
        .positionConversionFactor(1 / FuelConstants.PIVOT_GEAR_RATIO)
        .velocityConversionFactor(1 / FuelConstants.PIVOT_GEAR_RATIO);

        // stop config
        motor.setCANTimeout(0);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.current = motor.getOutputCurrent();
        inputs.position = encoder.getPosition();
        inputs.velocity = Rotations.per(Minute).of(encoder.getVelocity()).in(RotationsPerSecond);
        inputs.temperature = motor.getMotorTemperature();

        inputs.positionSetpoint = positionSetpoint.in(Rotations);
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
            FuelConstants.PIVOT_KCOS * Math.cos(Rotations.of(encoder.getPosition()).in(Radians))
        );

        positionSetpoint = angle;
    }
}