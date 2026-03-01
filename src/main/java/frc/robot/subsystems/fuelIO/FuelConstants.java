package frc.robot.subsystems.fuelIO;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;

public class FuelConstants {
    
    // ————— CAN ids ————— //
    
    public static final int INTAKE_MOTOR_ID = 50;
    public static final int PIVOT_MOTOR_ID = 51;
    public static final int HOPPER_MOTOR_ID = 52;
    public static final int SHOOTER_MOTOR_ID = 53;
    public static final int HOOD_MOTOR_ID = 54;
    public static final int KICKER_MOTOR_ID = 55;
    public static final int CLIMBER_MOTOR_ID = 56;

    // ————— physical constants ————— //

    // gear ratios are all # rotations of motor to get one rotation of the mechanism
    public static final double INTAKE_GEAR_RATIO = 5.0; // kind of incorrect, but no one cares
    public static final double PIVOT_GEAR_RATIO = 25.0 / 84.0 * 50.0;
    public static final double HOPPER_GEAR_RATIO = 4.0;
    public static final double SHOOTER_GEAR_RATIO = 1;
    public static final double HOOD_GEAR_RATIO = 64.0 / 30.0 * 60.0;
    public static final double KICKER_GEAR_RATIO = 1;
    
    // when pivot is zeroed at horizontal
    public static final Angle PIVOT_UP_ANGLE = Rotations.of(0.2); // !
    public static final Angle PIVOT_DOWN_ANGLE = Rotations.of(-0.0976); // ! 

    // ————— PIDF ————— //

    public static final ClosedLoopConfig PIVOT_PID = new ClosedLoopConfig().pid(10, 0, 0); // ! needs tuning
    public static final double PIVOT_KCOS = 0.45; // ! needs tuning

    public static final Slot0Configs SHOOTER_PIDF = new Slot0Configs() // ! needs tuning
    .withKP(0.3)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0.13259)
    .withKA(0);

    public static final Slot0Configs HOOD_PIDF = new Slot0Configs() // ! needs tuning
    .withKP(70)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0)
    .withKA(0);

    // ————— sim ————— //
    
    public static final Distance INTAKE_WIDTH_X = Inches.of(11.5); // ! 
    public static final int HOPPER_FUEL_CAPACITY = 30; // ! 
    public static final Transform3d SHOOTER_POSITION = new Transform3d(
        Inches.of(11), 
        Inches.of(0), 
        Inches.of(18), 
        new Rotation3d()
    ); // ! 
    public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2);
}