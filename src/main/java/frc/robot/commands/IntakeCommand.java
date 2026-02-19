package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;

public class IntakeCommand extends Command {
    private final Intake intake;
    
    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.setIntakeVoltage(Volts.of(12));
        intake.setPivotPosition(FuelConstants.PIVOT_DOWN_ANGLE);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeVoltage(Volts.of(0));
    }
}