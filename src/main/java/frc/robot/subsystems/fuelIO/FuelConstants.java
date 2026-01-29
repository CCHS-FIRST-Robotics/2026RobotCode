package frc.robot.subsystems.fuelIO;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FuelConstants {
    // ————— CAN ids ————— //
    public static final int INTAKE_MOTOR_ID = 50;
    public static final int PIVOT_MOTOR_ID = 51;
    public static final int HOPPER_MOTOR_ID = 52;
    public static final int TURRET_MOTOR_ID = 53;
    public static final int ANGLER_MOTOR_ID = 54;
    public static final int SHOOTER_MOTOR_ID = 55;

    // ————— physical constants ————— //
    public static final double INTAKE_GEAR_RATIO = 5;
    public static final double PIVOT_GEAR_RATIO = 25 * 84 / 50; // rotations of motor to get one rotation of the pivot

    // ————— PIDF ————— //

    public static final ClosedLoopConfig INTAKE_PID = new ClosedLoopConfig().pid(0.3, 0, 0);
    public static final double INTAKE_KS = 0;
    public static final double INTAKE_KV = 0.1;
    public static final double INTAKE_KA = 0;

    public static final ClosedLoopConfig PIVOT_PID = new ClosedLoopConfig().pid(0.05, 0, 0);
    public static final double PIVOT_KG = 1;

    public static final Slot0Configs SHOOTER_PIDF = new Slot0Configs()
    .withKP(0.3)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0.1)
    .withKA(0);

    // the interpolated map should probably exist here
}