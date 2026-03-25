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

    public static enum BUTTON_BINDINGS {
        COMPETITION,
        TESTING_BPS,
        TESTING_SHOOTER_MAP
    }

    // ————— initial conditions ————— //

    public static final Pose2d ROBOT_START_POSE = CURRENT_MODE == ROBOT_MODE.SIM ?
    new Pose2d(3, 3, new Rotation2d()) : 
    new Pose2d(0, 0, new Rotation2d());

    public static final BUTTON_BINDINGS CURRENT_BUTTON_BINDINGS = BUTTON_BINDINGS.COMPETITION;

    public static final boolean INSTANTIATE_DRIVE_AND_POSEESTIMATOR = false;
    public static final boolean INSTANTIATE_INTAKE = false;
    public static final boolean INSTANTIATE_SHOOTER = false;
    public static final boolean INSTANTIATE_LEDS = true;

    // real

    public static final Angle PIVOT_START_ANGLE = FuelConstants.PIVOT_MAX_UP_ANGLE;

    // sim

    public static final boolean REALISTIC_SIM = false;

    // ————— toggles ————— //

    public static boolean TRENCH_ALIGN = true;
    public static boolean USE_PIVOT = true;
    public static boolean SHOOT_ON_THE_MOVE = true;

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
        public static final Distance TRENCH_ZONE_WIDTH_X = Meters.of(1.5);

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

                public Translation2d[] getCorners() {
                    return new Translation2d[] {
                        new Translation2d(xMin, yMin),
                        new Translation2d(xMax, yMin),
                        new Translation2d(xMax, yMax),
                        new Translation2d(xMin, yMax),
                        new Translation2d(xMin, yMin) // bottom left is mirrored to form a closed loop
                    };
                }
            }

            public static class ZoneCollection {
                public final Zone[] zones;

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

            private static final Zone BLUE_BOTTOM_TRENCH_ENTERING = new Zone(
                FieldConstants.TRENCH_DISTANCE_X
                .minus(TRENCH_ZONE_WIDTH_X), 
                FieldConstants.TRENCH_DISTANCE_X
                .plus(TRENCH_ZONE_WIDTH_X), 
                Meters.of(0),
                FieldConstants.TRENCH_WIDTH_Y.plus(Meters.of(0.5))
            );
            private static final Zone BLUE_TOP_TRENCH_ENTERING = BLUE_BOTTOM_TRENCH_ENTERING.mirroredY();
            private static final Zone RED_BOTTOM_TRENCH_ENTERING = BLUE_BOTTOM_TRENCH_ENTERING.mirroredX();
            private static final Zone RED_TOP_TRENCH_ENTERING = BLUE_TOP_TRENCH_ENTERING.mirroredX();

            public static final ZoneCollection TRENCH_ZONES_ENTERING = new ZoneCollection(
                BLUE_BOTTOM_TRENCH_ENTERING, 
                BLUE_TOP_TRENCH_ENTERING, 
                RED_BOTTOM_TRENCH_ENTERING, 
                RED_TOP_TRENCH_ENTERING
            );

            private static final Zone BLUE_BOTTOM_TRENCH_EXITING = new Zone(
                FieldConstants.TRENCH_DISTANCE_X
                .minus(TRENCH_ZONE_WIDTH_X.minus(Meters.of(0.5))), 
                FieldConstants.TRENCH_DISTANCE_X
                .plus(TRENCH_ZONE_WIDTH_X.minus(Meters.of(0.5))), 
                Meters.of(0),
                FieldConstants.TRENCH_WIDTH_Y
            );
            private static final Zone BLUE_TOP_TRENCH_EXITING = BLUE_BOTTOM_TRENCH_EXITING.mirroredY();
            private static final Zone RED_BOTTOM_TRENCH_EXITING = BLUE_BOTTOM_TRENCH_EXITING.mirroredX();
            private static final Zone RED_TOP_TRENCH_EXITING = BLUE_TOP_TRENCH_EXITING.mirroredX();

            public static final ZoneCollection TRENCH_ZONES_EXITING = new ZoneCollection(
                BLUE_BOTTOM_TRENCH_EXITING, 
                BLUE_TOP_TRENCH_EXITING, 
                RED_BOTTOM_TRENCH_EXITING, 
                RED_TOP_TRENCH_EXITING
            );

            public static ZoneCollection TRENCH_ZONES = TRENCH_ZONES_ENTERING;

            public static void logAllZones() {
                Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/blue bottom", RED_TOP_TRENCH_ENTERING.getCorners());
                Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/blue top", RED_TOP_TRENCH_ENTERING.getCorners());
                Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/red bottom", RED_TOP_TRENCH_ENTERING.getCorners());
                Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/red top", RED_TOP_TRENCH_ENTERING.getCorners());
            }
        }
    }
}