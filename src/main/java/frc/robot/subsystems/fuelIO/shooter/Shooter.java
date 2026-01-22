package frc.robot.subsystems.fuelIO.shooter;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.fuleIO.shooter.ShooterIOInputsAutoLogged;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    boolean autoVelocity;

    public Shooter(ShooterIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("shooter", inputs);

        if (autoVelocity) {
            // ! smth smth interpolated map
        }
    }

    // ————— raw command factories ————— //

    public Command getSetShooterVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setVoltage(volts));
    }

    public Command getSetShooterVelocityCommand(AngularVelocity velocity) {
        // maybe stop autovelocity if it's running (idk when this command factory will actually be used though)
        return runOnce(() -> io.setVelocity(velocity));
    }

    // ————— sysid command factories ————— //


}