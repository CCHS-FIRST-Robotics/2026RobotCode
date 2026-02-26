package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

public class Controller extends CommandXboxController{
    private final double DEADBAND = 0.1;

    public Controller(int port) {
        super(port);
    }

    public double getLeftXWithDeadband() {
        return MathUtil.applyDeadband(super.getLeftX(), DEADBAND);
    }

    public double getLeftYWithDeadband() {
        return MathUtil.applyDeadband(super.getLeftY(), DEADBAND);
    }

    public double getRightXWithDeadband() {
        return MathUtil.applyDeadband(super.getRightX(), DEADBAND);
    }

    public double getRightYWithDeadband() {
        return MathUtil.applyDeadband(super.getRightY(), DEADBAND);
    }
}