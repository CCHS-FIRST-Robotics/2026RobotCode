package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("subsystems/climber", inputs);
    }

    // ————— raw command factories ————— //

    // intake

    public void setClimberVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    public Command getSetClimberVoltageCommand(Voltage volts) {
        return runOnce(() -> setClimberVoltage(volts));
    }
}