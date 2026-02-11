package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooterIO;
    private final ShooterIOInputsAutoLogged shooterIOInputs = new ShooterIOInputsAutoLogged();

    private final AnglerIO anglerIO;
    private final AnglerIOInputsAutoLogged anglerIOInputs = new AnglerIOInputsAutoLogged();

    private final KickerIO kickerIO;
    private final KickerIOInputsAutoLogged kickerIOInputs = new KickerIOInputsAutoLogged();

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
        Logger.processInputs("shooter", shooterIOInputs);
        anglerIO.updateInputs(anglerIOInputs);
        Logger.processInputs("angler", anglerIOInputs);
        kickerIO.updateInputs(kickerIOInputs);
        Logger.processInputs("kicker", kickerIOInputs);
    }

    // ————— raw command factories ————— //

    public Command getSetShooterVoltageCommand(Voltage volts) {
        return runOnce(() -> shooterIO.setShooterVoltage(volts));
    }

    public Command getSetShooterVelocityCommand(AngularVelocity velocity) {
        return runOnce(() -> shooterIO.setShooterVelocity(velocity));
    }

    public Command getSetAnglerVoltageCommand(Voltage volts) {
        return runOnce(() -> anglerIO.setAnglerVoltage(volts));
    }

    public Command getSetAnglerPositionCommand(Angle angle) {
        return runOnce(() -> anglerIO.setAnglerPosition(angle));
        // ! erm pivot code
    }

    public Command getSetKickerVoltageCommand(Voltage volts) {
        return runOnce(() -> kickerIO.setKickerVoltage(volts));
    }

    // ————— sysid command factories ————— //
}