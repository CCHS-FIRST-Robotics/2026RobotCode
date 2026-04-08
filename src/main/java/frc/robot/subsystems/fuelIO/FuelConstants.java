package frc.robot.subsystems.fuelIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;

public class FuelConstants {
    
    // ————— CAN ids ————— //
    
    public static final int INTAKE_MOTOR_ID = 50;
    public static final int PIVOT_MOTOR_ID = 51;
    public static final int SHOOTER_MOTOR_ID = 52;
    public static final int SHOOTER_FOLLOWER_ID = 53;
    public static final int KICKER_MOTOR_ID = 54;

    // ————— physical constants ————— //

    // gear ratios are all # rotations of motor to get one rotation of the mechanism
    public static final double INTAKE_GEAR_RATIO = 5.0; // kind of incorrect, but no one cares
    public static final double PIVOT_GEAR_RATIO = 75.0 / 84.0 * 50.0;
    public static final double SHOOTER_GEAR_RATIO = 1;
    public static final double KICKER_GEAR_RATIO = 1;
    
    // when pivot is zeroed at horizontal
    public static final Angle PIVOT_MAX_UP_ANGLE = Rotations.of(0.37);
    public static final Angle PIVOT_MAX_DOWN_ANGLE = Rotations.of(-0.056);

    // ————— PIDF ————— //

    public static final ClosedLoopConfig PIVOT_PID = new ClosedLoopConfig().pid(10, 0, 0);
    public static final FeedForwardConfig PIVOT_FF = new FeedForwardConfig().kCos(0.5);

    public static final Slot0Configs SHOOTER_PIDF = new Slot0Configs()
    .withKP(0.3)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0.13259)
    .withKA(0);

    public static final ClosedLoopConfig KICKER_PID = new ClosedLoopConfig().pid(0.00001, 0, 0);
    public static final FeedForwardConfig KICKER_FF = new FeedForwardConfig().kV(0.0021); // this is in RPM

    // ————— sim ————— //
    
    public static final Distance INTAKE_WIDTH_X = Inches.of(9.6);
    public static final int HOPPER_FUEL_CAPACITY = 50; // ! 
    public static final Transform3d SHOOTER_POSITION = new Transform3d(
        Inches.of(-9.2), 
        Inches.of(0), 
        Inches.of(12.5), 
        new Rotation3d()
    );
    public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2);
    public static final Angle HOOD_ANGLE = Degrees.of(10); // where 0 is with the hood horizontal (ball shoots straight up)
}