/**
 * Original code
 */

package frc.robot.subsystems.fuelIO.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.*;
import frc.robot.utils.ShootUtil;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    public final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    private final KickerIO kickerIO;
    public final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

    private AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    private AngularVelocity kickerVelocity = RotationsPerSecond.of(0);

    public Shooter(
        ShooterIO shooterIO, 
        KickerIO kickerIO
    ) {
        this.shooterIO = shooterIO;
        this.kickerIO = kickerIO;
    }
    
    @Override
    public void periodic() {
        shooterIO.updateInputs(shooterIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/shooter", shooterIOInputs);
        kickerIO.updateInputs(kickerIOInputs);
        Logger.processInputs("subsystems/fuelIO/shooter/kicker", kickerIOInputs);

        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) { // sim PIDs must be called externally
            shooterIO.setVelocity(shooterVelocity);
        }

        kickerIO.setVelocity(kickerVelocity); // NEO and sim PIDs must be called externally
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

    // kicker

    public void setKickerVoltage(Voltage volts) {
        kickerIO.setVoltage(volts);
    }

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> setKickerVoltage(volts));
    }

    public void setKickerVelocity(AngularVelocity velocity) {
        kickerVelocity = velocity;
    }

    public Command getSetKickerVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> setKickerVelocity(velocity));
    }

    // util

    @AutoLogOutput(key = "outputs/fuelIO/shooter/shooterUpToSpeed")
    public boolean getShooterUpToSpeed() {
        return Math.abs(shooterIOInputs.velocity - shooterIOInputs.velocitySetpoint) < 5;
    }

    public double getShooterAngularVelocity() {
        return shooterIOInputs.velocity;
    }

    @AutoLogOutput(key = "outputs/fuelIO/shooter/linearVelocity")
    public LinearVelocity getShooterLinearVelocity() {
        return ShootUtil.calculateShooterLinearVelocity(RotationsPerSecond.of(shooterIOInputs.velocity));
    }
}