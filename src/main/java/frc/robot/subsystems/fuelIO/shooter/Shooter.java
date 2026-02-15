package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    private final AnglerIO anglerIO;
    private final AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

    AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    Angle anglerAngle = Constants.ANGLER_START_ANGLE;

    public Shooter(
        ShooterIO shooterIO, 
        AnglerIO anglerIO, 
        KickerIO kickerIO
    ) {
        this.shooterIO = shooterIO;
        this.anglerIO = anglerIO;
        this.kickerIO = kickerIO;
    }
    
    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/shooter", shooterIOInputs);
        anglerIO.updateInputs(anglerIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/angler", anglerIOInputs);
        kickerIO.updateInputs(kickerIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/kicker", kickerIOInputs);

        if(Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) {
            shooterIO.setVelocity(shooterVelocity);
        }

        if(Constants.ENABLE_ANGLER_SET_POSITION) {
            anglerIO.setPosition(anglerAngle);
        }
    }

    // ————— raw command factories ————— //

    public Command getSetShooterVoltageCommand(Voltage volts) {
        return runOnce(() -> shooterIO.setVoltage(volts));
    }

    public Command getSetShooterVelocityCommand(AngularVelocity velocity) {
        return Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL ? 
            runOnce(() -> shooterIO.setVelocity(velocity)) : 
            runOnce(() -> shooterVelocity = velocity);
    }

    // ! do not call if anglerIO.setPosition() is active
    public Command getSetAnglerVoltageCommand(Voltage volts) {
        return runOnce(() -> anglerIO.setVoltage(volts));
    }

    public Command getSetAnglerPositionCommand(Angle angle) {
        return runOnce(() -> anglerAngle = angle);
    }

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> kickerIO.setVoltage(volts));
    }

    public double getShooterAngularVelocity() {
        return shooterIOInputs.velocity;
    }

    public LinearVelocity getShooterLinearVelocity() {
        double angularVelocity = RotationsPerSecond.of(shooterIOInputs.velocity).in(RadiansPerSecond);
        return InchesPerSecond.of(angularVelocity * 2);  // multiply by shooter wheel radius
    }

    public Angle getAnglerAngle() {
        return Rotations.of(anglerIOInputs.position);
    }

    // ————— sysid command factories ————— //
}