package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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
    public static final Angle HOOD_START_ANGLE = FuelConstants.HOOD_DOWN_ANGLE;
    
    public static final boolean ENABLE_PIVOT_SET_POSITION = true;
    public static final boolean ENABLE_HOOD_SET_POSITION = true;

    public static class FieldConstants {
        public static final Distance FIELD_WIDTH_X = Inches.of(650.12);
        public static final Distance FIELD_WIDTH_Y = Inches.of(316.64);
        
        public static final Distance ALLIANCE_ZONE_WIDTH_X = Inches.of(156.61);

        public static final Pose3d BLUE_HUB = new Pose3d(
            Inches.of(181.56), 
            FIELD_WIDTH_Y.div(2), 
            Inches.of(56.4),
            new Rotation3d()
        );
        public static final Pose3d RED_HUB = new Pose3d(
            FIELD_WIDTH_X.minus(Inches.of(181.56)), 
            FIELD_WIDTH_Y.div(2), 
            Inches.of(56.4),
            new Rotation3d()
        );

        // ! 5000 just uses a translation which is smart and also this is wrong and doesn't work
        public static final Pose2d BLUE_PASS = new Pose2d(
            Inches.of(181.56), 
            FIELD_WIDTH_Y.div(2), 
            new Rotation2d()
        );
        public static final Pose2d RED_PASS = new Pose2d(
            FIELD_WIDTH_X.minus(Inches.of(181.56)), 
            FIELD_WIDTH_Y.div(2), 
            new Rotation2d()
        );
    }
}