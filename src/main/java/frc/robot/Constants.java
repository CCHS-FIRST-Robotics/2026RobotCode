package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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

    public static final int CONTROLLER_PORT = 0;
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

    public static final boolean INSTANTIATE_DRIVE_AND_POSEESTIMATOR = true;
    public static final boolean INSTANTIATE_INTAKE = true;
    public static final boolean INSTANTIATE_SHOOTER = true;

    // real

    public static final Angle PIVOT_START_ANGLE = FuelConstants.PIVOT_MAX_UP_ANGLE;

    // sim

    public static final boolean REALISTIC_SIM = false;

    // ————— toggles ————— //

    public static boolean ENABLE_TRENCH_ALIGN = true;
    public static boolean ENABLE_PIVOT = true;
    public static boolean ENABLE_PIVOT_AGITATION = true;
    public static boolean ENABLE_SHOOT_ON_THE_MOVE = false;

    public static class FieldConstants {
        public static final Field2d FIELD2D = new Field2d();

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
                public final String name;
                public final double xMin, xMax, yMin, yMax;

                public Zone(String name, double xMin, double xMax, double yMin, double yMax) {
                    this.name = name;
                    this.xMin = xMin;
                    this.xMax = xMax;
                    this.yMin = yMin;
                    this.yMax = yMax;
                }

                public Zone(String name, Distance xMin, Distance xMax, Distance yMin, Distance yMax) {
                    this(name, xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
                }

                public boolean contains(Pose2d pose) {
                    return this.containsPoint(pose.getTranslation());
                }

                public boolean containsPoint(Translation2d point) {
                    return point.getX() >= xMin && point.getX() <= xMax && point.getY() >= yMin && point.getY() <= yMax;
                }

                public Zone mirroredX(String name) {
                    return new Zone(
                        name, 
                        FieldConstants.FIELD_WIDTH_X.in(Meters) - xMax,
                        FieldConstants.FIELD_WIDTH_X.in(Meters) - xMin,
                        yMin,
                        yMax
                    );
                }

                public Zone mirroredY(String name) {
                    return new Zone(
                        name, 
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
                        new Translation2d(xMin, yMin) // bottom left is repeated to form a closed loop
                    };
                }
            }

            public static class ZoneCollection {
                public final String name;
                public final Zone[] zones;

                public ZoneCollection(String name, Zone... zones) {
                    this.name = name;
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

                public void log() {
                    for (Zone z : zones) {
                        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/" + name + "/" + z.name, z.getCorners());
                    }
                }
            }

            // ————— default ————— //

            private static final Zone BLUE_LEFT_TRENCH_DEFAULT = new Zone(
                "blue left",
                FieldConstants.TRENCH_DISTANCE_X
                .minus(TRENCH_ZONE_WIDTH_X),
                FieldConstants.TRENCH_DISTANCE_X
                .plus(TRENCH_ZONE_WIDTH_X),
                FieldConstants.FIELD_WIDTH_Y
                .minus(TRENCH_WIDTH_Y)
                .minus(Meters.of(0.5)),
                FieldConstants.FIELD_WIDTH_Y
            );
            private static final Zone BLUE_RIGHT_TRENCH_DEFAULT = BLUE_LEFT_TRENCH_DEFAULT.mirroredY("blue right");
            private static final Zone RED_LEFT_TRENCH_DEFAULT = BLUE_RIGHT_TRENCH_DEFAULT.mirroredX("red left");
            private static final Zone RED_RIGHT_TRENCH_DEFAULT = BLUE_LEFT_TRENCH_DEFAULT.mirroredX("red right");

            public static final ZoneCollection TRENCH_ZONES_DEFAULT = new ZoneCollection( // big
                "trenches/default",
                BLUE_LEFT_TRENCH_DEFAULT, 
                BLUE_RIGHT_TRENCH_DEFAULT, 
                RED_LEFT_TRENCH_DEFAULT, 
                RED_RIGHT_TRENCH_DEFAULT
            );

            // ————— alliance ————— //

            private static final Zone BLUE_LEFT_TRENCH_ALLIANCE = new Zone(
                "blue left",
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.xMin),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.xMax).minus(Meters.of(1)),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.yMin),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.yMax)
            );
            private static final Zone BLUE_RIGHT_TRENCH_ALLIANCE = BLUE_LEFT_TRENCH_ALLIANCE.mirroredY("blue right");
            private static final Zone RED_LEFT_TRENCH_ALLIANCE = BLUE_RIGHT_TRENCH_ALLIANCE.mirroredX("red left");
            private static final Zone RED_RIGHT_TRENCH_ALLIANCE = BLUE_LEFT_TRENCH_ALLIANCE.mirroredX("red right");

            public static final ZoneCollection TRENCH_ZONES_ALLIANCE = new ZoneCollection( // entering from alliance side
                "trenches/alliance",
                BLUE_LEFT_TRENCH_ALLIANCE, 
                BLUE_RIGHT_TRENCH_ALLIANCE, 
                RED_LEFT_TRENCH_ALLIANCE, 
                RED_RIGHT_TRENCH_ALLIANCE
            );

            // ————— neutral ————— //

            private static final Zone BLUE_LEFT_TRENCH_NEUTRAL = new Zone(
                "blue left",
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.xMin).plus(Meters.of(1)),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.xMax),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.yMin),
                Meters.of(BLUE_LEFT_TRENCH_DEFAULT.yMax)
            );
            private static final Zone BLUE_RIGHT_TRENCH_NEUTRAL = BLUE_LEFT_TRENCH_NEUTRAL.mirroredY("blue right");
            private static final Zone RED_LEFT_TRENCH_NEUTRAL = BLUE_RIGHT_TRENCH_NEUTRAL.mirroredX("red left");
            private static final Zone RED_RIGHT_TRENCH_NEUTRAL = BLUE_LEFT_TRENCH_NEUTRAL.mirroredX("red right");

            public static final ZoneCollection TRENCH_ZONES_NEUTRAL = new ZoneCollection( // entering from neutral side
                "trenches/neutral",
                BLUE_LEFT_TRENCH_NEUTRAL, 
                BLUE_RIGHT_TRENCH_NEUTRAL, 
                RED_LEFT_TRENCH_NEUTRAL, 
                RED_RIGHT_TRENCH_NEUTRAL
            );

            public static ZoneCollection TRENCH_ZONES = TRENCH_ZONES_DEFAULT;

            public static void logAllZones() {
                TRENCH_ZONES_DEFAULT.log();
                TRENCH_ZONES_ALLIANCE.log();
                TRENCH_ZONES_NEUTRAL.log();
            }
        }
    }
}