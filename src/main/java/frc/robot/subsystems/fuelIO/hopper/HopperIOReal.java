package frc.robot.subsystems.fuelIO.hopper;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HopperIOReal implements HopperIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;
    
    private final SimpleMotorFeedforward feedforward;

    private AngularVelocity velocitySetpoint = RotationsPerSecond.of(0);

    public HopperIOReal(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        encoder.setPosition(0.0);

        // pid 
        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).apply(FuelConstants.HOPPER_PID);
        feedforward = FuelConstants.HOPPER_FF;
        
        // miscellaneous settings
        motorConfig.signals.primaryEncoderVelocityPeriodMs(10);
        motorConfig.encoder.quadratureMeasurementPeriod(10);
        motorConfig.encoder.quadratureAverageDepth(2);
        
        motorConfig.smartCurrentLimit(30);
        motorConfig.voltageCompensation(12);
        
        motorConfig.inverted(true);
        
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
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.current = motor.getOutputCurrent();
        inputs.position = encoder.getPosition(); // Now in motor rotations (adjusted by gear ratio)
        inputs.velocity = encoder.getVelocity(); // Now in RPM (adjusted by gear ratio)
        inputs.temperature = motor.getMotorTemperature();

        inputs.velocitySetpoint = velocitySetpoint.in(RotationsPerSecond);
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.getClosedLoopController().setSetpoint(
            velocity.in(RPM), 
            SparkMax.ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0,
            feedforward.calculateWithVelocities(velocitySetpoint.in(RotationsPerSecond), velocity.in(RotationsPerSecond))
        );

        velocitySetpoint = velocity;
    }
}