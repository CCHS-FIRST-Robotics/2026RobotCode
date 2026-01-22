package frc.robot.subsystems.fuelIO;

import com.ctre.phoenix6.configs.Slot0Configs;

public class FuelConstants {
    public final int SHOOTER_MOTOR_ID = 50; // ! probably start from 50 for less confusion


    public static final Slot0Configs SHOOTER_PID = new Slot0Configs()
    .withKP(0.3)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0.1)
    .withKA(0);



    // the interpolated map should probably exist here
}
