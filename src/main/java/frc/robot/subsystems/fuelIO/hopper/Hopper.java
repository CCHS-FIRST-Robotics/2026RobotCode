package frc.robot.subsystems.fuelIO.hopper;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.fuelIO.FuelConstants;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.*;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    public final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    private int numFuel;

    public Hopper(HopperIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("subsystems/fuelIO/hopper", inputs);
    }

    // ————— raw command factories ————— //

    // hopper

    public void setHopperVoltage(Voltage volts) {
        io.setVoltage(volts);
    }

    public Command getSetHopperVoltageCommand(Voltage volts) {
        return runOnce(() -> setHopperVoltage(volts));
    }

    // util

    public void intakeFuel() {
        numFuel++;
    }

    public void shootFuel() {
        numFuel--;
    }

    @AutoLogOutput(key = "outputs/fuelIO/hopper/numFuel")
    public int getNumFuel() {
        return numFuel;
    }

    public boolean getHopperFull() {
        return numFuel >= FuelConstants.HOPPER_FUEL_CAPACITY;
    }

    public boolean getHopperEmpty() {
        return numFuel <= 0;
    }
}