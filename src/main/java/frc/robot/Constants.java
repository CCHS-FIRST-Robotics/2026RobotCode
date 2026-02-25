package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = RobotBase.isReal() ? ROBOT_MODE.REAL : ROBOT_MODE.SIM;

    public static final boolean REALISTIC_SIM = true;

    public static final boolean USE_ALLIANCE_FLIPPING = false; // ! 

    public static final int CONTROLLER_PORT = 0;
    
    public static final Pose2d ROBOT_START_POSE = CURRENT_MODE == ROBOT_MODE.SIM ?
    new Pose2d(3, 3, new Rotation2d()) : 
    new Pose2d(0, 0, new Rotation2d());

    public static final Angle PIVOT_START_ANGLE = FuelConstants.PIVOT_DOWN_ANGLE; // !
    public static final Angle HOOD_START_ANGLE = Rotations.of(0);
    
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