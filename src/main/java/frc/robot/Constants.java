package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = RobotBase.isReal() ? ROBOT_MODE.REAL : ROBOT_MODE.SIM;

    public static final boolean USE_ALLIANCE_FLIPPING = false;

    public static final int CONTROLLER_PORT_1 = 0;
}