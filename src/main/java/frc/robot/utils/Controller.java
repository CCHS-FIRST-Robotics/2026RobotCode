/**
 * Original code
 */

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;

public class Controller extends CommandXboxController{
    private final double DEFAULT_DEADBAND = 0.1;

    public Controller(int port) {
        super(port);
    }

    public double getLeftXWithDeadband(double deadband) {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    public double getLeftXWithDeadband() {
        return this.getLeftXWithDeadband(DEFAULT_DEADBAND);
    }

    public double getLeftYWithDeadband(double deadband) {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    public double getLeftYWithDeadband() {
        return this.getLeftYWithDeadband(DEFAULT_DEADBAND);
    }

    public double getRightXWithDeadband(double deadband) {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    public double getRightXWithDeadband() {
        return this.getRightXWithDeadband(DEFAULT_DEADBAND);
    }

    public double getRightYWithDeadband(double deadband) {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
    }

    public double getRightYWithDeadband() {
        return this.getRightYWithDeadband(DEFAULT_DEADBAND);
    }
}