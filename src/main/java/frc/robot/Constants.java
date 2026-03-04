package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.fuelIO.*;

public final class Constants {
    public static final double PERIOD = 0.02;
    public static enum ROBOT_MODE {
        REAL,
        SIM,
        REPLAY
    }
    public static final ROBOT_MODE CURRENT_MODE = RobotBase.isReal() ? ROBOT_MODE.REAL : ROBOT_MODE.SIM;

    public static final boolean REALISTIC_SIM = false;
    
    public static final Pose2d ROBOT_START_POSE = CURRENT_MODE == ROBOT_MODE.SIM ?
    new Pose2d(3, 3, new Rotation2d()) : 
    new Pose2d(0, 0, new Rotation2d()); // in real, it gets reset immediately anyways

    public static final int CONTROLLER_PORT = 0;

    public static final Angle PIVOT_START_ANGLE = FuelConstants.PIVOT_MAX_DOWN_ANGLE; // ! 
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

        public static final Pose2d BLUE_PASS_LEFT = new Pose2d(
            Meters.of(2), 
            FIELD_WIDTH_Y.div(2).plus(Meters.of(2)), 
            new Rotation2d()
        );
        public static final Pose2d BLUE_PASS_RIGHT = new Pose2d(
            Meters.of(2), 
            FIELD_WIDTH_Y.div(2).minus(Meters.of(2)), 
            new Rotation2d()
        );

        public static final Distance TRENCH_DISTANCE_X = Inches.of(182.11);
        public static final Distance TRENCH_WIDTH_Y = Inches.of(50.59);
        public static final Distance TRENCH_ZONE_WIDTH_X = Meters.of(1);

        public static Pose2d calculateAllianceFlippedPose(Pose2d pose) {
            return pose.rotateAround(
                new Translation2d(FieldConstants.FIELD_WIDTH_X.div(2), FieldConstants.FIELD_WIDTH_Y.div(2)), 
                new Rotation2d(Degrees.of(180))
        );
    }

        public class Zones {
            public static class Zone {
                protected final double xMin, xMax, yMin, yMax;

                public Zone(double xMin, double xMax, double yMin, double yMax) {
                    this.xMin = xMin;
                    this.xMax = xMax;
                    this.yMin = yMin;
                    this.yMax = yMax;
                }

                public Zone(Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
                    this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
                }

                public boolean contains(Pose2d pose) {
                    return this.containsPoint(pose.getTranslation());
                }

                protected boolean containsPoint(Translation2d point) {
                    return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
                }

                public Zone mirroredX() {
                    return new Zone(
                        FieldConstants.FIELD_WIDTH_X.in(Meters) - xMax,
                        FieldConstants.FIELD_WIDTH_X.in(Meters) - xMin,
                        yMin,
                        yMax
                    );
                }

                public Zone mirroredY() {
                    return new Zone(
                        xMin,
                        xMax,
                        FieldConstants.FIELD_WIDTH_Y.in(Meters) - yMax,
                        FieldConstants.FIELD_WIDTH_Y.in(Meters) - yMin
                    );
                }

                /** list of corners, with the bottom left corner repeated at the end to form a closed loop */
                public Translation2d[] getCorners() {
                    return new Translation2d[] {
                        new Translation2d(xMin, yMin),
                        new Translation2d(xMax, yMin),
                        new Translation2d(xMax, yMax),
                        new Translation2d(xMin, yMax),
                        new Translation2d(xMin, yMin)
                    };
                }
            }

            public static class ZoneCollection {
                protected final Zone[] zones;

                public ZoneCollection(Zone... zones) {
                    this.zones = zones;
                }

                public boolean contains(Pose2d pose) {
                    for (Zone zone : zones) {
                        if (zone.contains(pose)) {
                            return true;
                        }
                    }

                    return false;
                }
            }

            private static final Zone BLUE_BOTTOM_TRENCH = new Zone(
                FieldConstants.TRENCH_DISTANCE_X
                .minus(TRENCH_ZONE_WIDTH_X), 
                FieldConstants.TRENCH_DISTANCE_X
                .plus(TRENCH_ZONE_WIDTH_X), 
                Meters.of(0),
                FieldConstants.TRENCH_WIDTH_Y
            );
            private static final Zone BLUE_TOP_TRENCH = BLUE_BOTTOM_TRENCH.mirroredY();
            private static final Zone RED_BOTTOM_TRENCH = BLUE_BOTTOM_TRENCH.mirroredX();
            private static final Zone RED_TOP_TRENCH = BLUE_TOP_TRENCH.mirroredX();

            public static final ZoneCollection TRENCH_ZONES = new ZoneCollection(
                BLUE_BOTTOM_TRENCH, 
                BLUE_TOP_TRENCH, 
                RED_BOTTOM_TRENCH, 
                RED_TOP_TRENCH
            );

            public static void logAllZones() {
                Logger.recordOutput("Zones/Trenches/Blue Bottom", BLUE_BOTTOM_TRENCH.getCorners());
                Logger.recordOutput("Zones/Trenches/Blue Top", BLUE_TOP_TRENCH.getCorners());
                Logger.recordOutput("Zones/Trenches/Red Bottom", RED_BOTTOM_TRENCH.getCorners());
                Logger.recordOutput("Zones/Trenches/Red Top", RED_TOP_TRENCH.getCorners());
            }
        }
    }
}