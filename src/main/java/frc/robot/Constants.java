package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.fuelIO.FuelConstants;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = RobotBase.isReal() ? ROBOT_MODE.REAL : ROBOT_MODE.SIM;

    public static final boolean USE_ALLIANCE_FLIPPING = false; // !
    // ! also something for which hub to autoaim to

    public static final int CONTROLLER_PORT = 0; // 0 for gamesir, 1 for xbox

    public static final Angle PIVOT_START_ANGLE = FuelConstants.PIVOT_DOWN_ANGLE; // ! should be changed for comp
    public static final Angle ANGLER_START_ANGLE = FuelConstants.ANGLER_DOWN_ANGLE;
    
    public static final boolean ENABLE_PIVOT_SET_POSITION = true;
    public static final boolean ENABLE_ANGLER_SET_POSITION = true;
}