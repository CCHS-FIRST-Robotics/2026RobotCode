package frc.robot.subsystems.fuelIO.hopper;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private int numFuel;

    public Hopper(HopperIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("hopper", inputs);
    }

    // ————— raw command factories ————— //

    public Command getSetHopperVoltageCommand(Voltage volts) {
        return runOnce(() -> io.setHopperVoltage(volts));
    }

    public void intakeFuel() {
        numFuel++;
    }

    public void shootFuel() {
        numFuel--;
    }

    public int getNumFuel() {
        return numFuel;
    }

    // ————— processed command factories ————— //
}