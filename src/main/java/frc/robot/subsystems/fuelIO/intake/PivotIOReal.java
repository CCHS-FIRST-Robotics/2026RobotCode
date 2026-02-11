package frc.robot.subsystems.fuelIO.intake;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class PivotIOReal implements PivotIO {
    private final SparkMax pivotMotor;
    private final SparkMaxConfig pivotConfig = new SparkMaxConfig();
    private final RelativeEncoder pivotEncoder;

    public PivotIOReal(int id) {
        pivotMotor = new SparkMax(id, MotorType.kBrushless);

        // start config
        pivotMotor.setCANTimeout(500);

        // encoders
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(FuelConstants.INTAKE_START_ANGLE.in(Rotations));

        // pid 
        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.PIVOT_PID);
        pivotConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(0.25).in(Rotations.per(Minute)));
        pivotConfig.closedLoop.maxMotion.maxAcceleration(RotationsPerSecondPerSecond.of(100).in(Rotations.per(Minute).per(Second)));
        pivotConfig.closedLoop.maxMotion.allowedProfileError(Rotations.of(0.05).in(Rotations));

        // miscellaneous settings
        pivotConfig.signals.primaryEncoderVelocityPeriodMs(10);
        pivotConfig.encoder.quadratureMeasurementPeriod(10);
        pivotConfig.encoder.quadratureAverageDepth(2);
        
        pivotConfig.smartCurrentLimit(30);
        pivotConfig.voltageCompensation(12);
        
        pivotConfig.inverted(true); // !
        
        pivotConfig.idleMode(IdleMode.kCoast);

        pivotConfig.encoder
            .positionConversionFactor(1 / FuelConstants.PIVOT_GEAR_RATIO)
            .velocityConversionFactor(1 / FuelConstants.PIVOT_GEAR_RATIO);

        // stop config
        pivotMotor.setCANTimeout(0);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotVoltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        inputs.pivotCurrent = pivotMotor.getOutputCurrent();
        inputs.pivotPosition = pivotEncoder.getPosition();
        inputs.pivotVelocity = Rotations.per(Minute).of(pivotEncoder.getVelocity()).in(RotationsPerSecond);
        inputs.pivotTemperature = pivotMotor.getMotorTemperature();
    }

    @Override
    public void setPivotVoltage(Voltage volts) {
        pivotMotor.setVoltage(volts);
    }

    @Override
    public void setPivotPosition(Angle angle) {
        pivotMotor.getClosedLoopController().setSetpoint(
            angle.in(Rotations), 
            SparkMax.ControlType.kMAXMotionPositionControl, 
            ClosedLoopSlot.kSlot0,
            FuelConstants.PIVOT_KCOS * Math.cos(Rotations.of(pivotEncoder.getPosition()).in(Radians))
        );
    }
}