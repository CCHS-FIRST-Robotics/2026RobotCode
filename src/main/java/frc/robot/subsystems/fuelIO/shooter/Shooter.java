package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.utils.ShooterCalculator.ShooterState;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    private final HoodIO hoodIO;
    private final HoodIOInputsAutoLogged hoodIOInputs = new HoodIOInputsAutoLogged();

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

    private AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    private Angle hoodAngle = Constants.HOOD_START_ANGLE;

    public Shooter(
        ShooterIO shooterIO, 
        HoodIO hoodIO, 
        KickerIO kickerIO
    ) {
        this.shooterIO = shooterIO;
        this.hoodIO = hoodIO;
        this.kickerIO = kickerIO;
    }
    
    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/shooter", shooterIOInputs);
        hoodIO.updateInputs(hoodIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/hood", hoodIOInputs);
        kickerIO.updateInputs(kickerIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/kicker", kickerIOInputs);

        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) { // in sim, shooter PID needs to be called constantly
            shooterIO.setVelocity(shooterVelocity);
        }

        if (Constants.ENABLE_HOOD_SET_POSITION) {
            hoodIO.setPosition(hoodAngle);
        }
    }

    // ————— raw command factories ————— //

    public void setShooterVoltage(Voltage volts) {
        shooterIO.setVoltage(volts);
    }

    public Command getSetShooterVoltageCommand(Voltage volts) {
        return runOnce(() -> setShooterVoltage(volts));
    }

    public void setShooterVelocity(AngularVelocity velocity) {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            shooterIO.setVelocity(velocity);
        } else {
            shooterVelocity = velocity;
        }
    }

    public Command getSetShooterVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setShooterVelocity(velocity));
    }

    public void setHoodVoltage(Voltage volts) {
        hoodIO.setVoltage(volts);
    }

    public Command getSetHoodVoltageCommand(Voltage volts) {
        return runOnce(() -> setHoodVoltage(volts));
    }

    public void setHoodPosition(Angle angle) {
        hoodAngle = angle;
    }

    public Command getSetHoodPositionCommand(Angle angle) {
        return runOnce(() -> setHoodPosition(angle));
    }

    public void setKickerVoltage(Voltage volts) {
        kickerIO.setVoltage(volts);
    }

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> setKickerVoltage(volts));
    }

    public double getShooterAngularVelocity() {
        return shooterIOInputs.velocity;
    }

    public LinearVelocity getShooterLinearVelocity() {
        double angularVelocity = RotationsPerSecond.of(shooterIOInputs.velocity).in(RadiansPerSecond);
        return InchesPerSecond.of(angularVelocity * 2);  // multiply by shooter wheel radius
    }

    public Angle getHoodAngle() {
        return Rotations.of(hoodIOInputs.position);
    }

    // ————— processed command factories ————— //

    public void runShooterState(ShooterState state) {
        if(Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            shooterIO.setVelocity(state.velocity());
        } else {
            shooterVelocity = state.velocity();
        }

        hoodIO.setPosition(state.angle());
    }
}