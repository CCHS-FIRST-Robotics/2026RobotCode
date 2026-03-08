package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.*;

import frc.robot.utils.ShootUtil;
import frc.robot.utils.ShootUtil.ShooterState;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    public final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    private final HoodIO hoodIO;
    public final HoodIOInputsAutoLogged hoodIOInputs = new HoodIOInputsAutoLogged();

    private final KickerIO kickerIO;
    public final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

    private AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    private Angle hoodAngle = Rotations.of(0);

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

        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) { // sim PIDs must be called externally
            shooterIO.setVelocity(shooterVelocity);
            hoodIO.setPosition(hoodAngle);
        }
    }

    // ————— raw command factories ————— //

    // shooter

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

    // hood

    public void setHoodVoltage(Voltage volts) {
        hoodIO.setVoltage(volts);
    }

    public Command getSetHoodVoltageCommand(Voltage volts) {
        return runOnce(() -> setHoodVoltage(volts));
    }

    public void setHoodPosition(Angle angle) {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            hoodIO.setPosition(angle);
        } else {
            hoodAngle = angle;
        }
    }

    public Command getSetHoodPositionCommand(Angle angle) {
        return runOnce(() -> setHoodPosition(angle));
    }

    // kicker

    public void setKickerVoltage(Voltage volts) {
        kickerIO.setVoltage(volts);
    }

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> setKickerVoltage(volts));
    }

    // util

    public double getShooterAngularVelocity() {
        return shooterIOInputs.velocity;
    }

    @AutoLogOutput(key = "outputs/fuelIO/shooter/linearVelocity")
    public LinearVelocity getShooterLinearVelocity() {
        return ShootUtil.calculateShooterLinearVelocity(RotationsPerSecond.of(shooterIOInputs.velocity));
    }

    @AutoLogOutput(key = "outputs/fuelIO/shooter/hoodShotAngle")
    public Angle getHoodShotAngle() {
        return Rotations.of(0.25 - hoodIOInputs.position); // when hood is 0, it shoots vertically
    }

    // ————— processed command factories ————— //

    public void runShooterState(ShooterState state) {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            shooterIO.setVelocity(state.velocity());
        } else {
            shooterVelocity = state.velocity();
        }

        hoodIO.setPosition(state.angle());
    }
}