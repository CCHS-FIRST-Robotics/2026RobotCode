package frc.robot.subsystems.fuelIO.hopper;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.*;



import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class HopperIOReal implements HopperIO {
    private final SparkMax motor;
    private final SparkMaxConfig motorConfig = new SparkMaxConfig();
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    public HopperIOReal(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        motorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // You will need to tune these constants!
            .p(0.1) 
            .i(0.0)
            .d(0.0)
            .velocityFF(0.12); // Feed-forward is crucial for velocity

        // 2. Conversion Factors
        // Ensure these are in rotations (or RPS) so your setpoints make sense
        motorConfig.encoder
            .positionConversionFactor(1.0 / FuelConstants.HOPPER_GEAR_RATIO)
            .velocityConversionFactor(1.0 / FuelConstants.HOPPER_GEAR_RATIO);

        // General motor settings
        motorConfig.smartCurrentLimit(30);
        motorConfig.idleMode(SparkMaxConfig.IdleMode.kCoast);
        motorConfig.inverted(true);

        // Apply configuration
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.voltage = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.current = motor.getOutputCurrent();
        inputs.position = encoder.getPosition(); // Now in motor rotations (adjusted by gear ratio)
        inputs.velocity = encoder.getVelocity(); // Now in RPM (adjusted by gear ratio)
        inputs.temperature = motor.getMotorTemperature();

        inputs.velocitySetpoint = motor.getClosedLoopController().getSetpoint() / 60;
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        // SparkMax expects RPM by default for velocity control
        double velocityRPS = velocity.in(RotationsPerSecond);
        closedLoopController.setSetpoint(velocityRPS * 60, SparkMax.ControlType.kVelocity);
    }
}