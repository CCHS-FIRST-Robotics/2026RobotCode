/**
 * Based on https://github.com/hammerheads5000/2026Rebuilt/blob/main/src/main/java/frc/robot/subsystems/turret/TurretCalculator.java
 */

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ShootUtil {
    public static final InterpolatingDoubleTreeMap SHOOTER_VELOCITY_MAP = new InterpolatingDoubleTreeMap();

    static {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) { // measured using TESTING_SHOOTER_MAP controller bindings
            SHOOTER_VELOCITY_MAP.put(1.797, 42.14648437500001);
            SHOOTER_VELOCITY_MAP.put(2.380, 43.572265625);
            SHOOTER_VELOCITY_MAP.put(2.960, 46.43750000000001);
            SHOOTER_VELOCITY_MAP.put(3.546, 49.66210937500001);
            SHOOTER_VELOCITY_MAP.put(4.020, 53.64257812500001);
            SHOOTER_VELOCITY_MAP.put(4.884, 59.12499999999999);
        } else { // placeholder sim values (I never decided to get them)
            SHOOTER_VELOCITY_MAP.put(1.908, 46.539058922493794);
            SHOOTER_VELOCITY_MAP.put(2.998, 57.8593705522896);
            SHOOTER_VELOCITY_MAP.put(3.919, 65.40624497215343);
        }
    }

    // values to output
    private static AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    private static Rotation2d robotRotation = new Rotation2d();
    private static Distance targetDistance = Meters.of(0);

    // ————— public functions ————— //

    public static Pose2d getTargetPose(Pose2d robotPose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) { // convert robot pose to blue alliance reference frame (makes casework a lot easier)
            robotPose = FieldConstants.calculateAllianceFlippedPose(robotPose);
        }

        Pose2d targetPose = new Pose2d();

        if (robotPose.getX() < FieldConstants.ALLIANCE_ZONE_WIDTH_X.in(Meters)) { // hub
            targetPose = FieldConstants.BLUE_HUB.toPose2d();
        } else { // passing
            if (robotPose.getY() > FieldConstants.FIELD_WIDTH_Y.div(2).in(Meters)) { // passing left
                targetPose = FieldConstants.BLUE_PASS_LEFT;
            } else { // passing right
                targetPose = FieldConstants.BLUE_PASS_RIGHT;
            }
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return targetPose;
        } else {
            return FieldConstants.calculateAllianceFlippedPose(targetPose); // convert back to red alliance
        }
    }

    // used when robot is shooting while stationary
    public static void update(Pose2d robotPose, Pose2d targetPose) {
        targetDistance = calculateRobotToTargetDistance(robotPose, targetPose);
        shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
        robotRotation = calculateRobotRotationToTarget(robotPose, targetPose);

        Logger.recordOutput("outputs/fuelIO/shooter/targetPose", targetPose);
        Logger.recordOutput("outputs/fuelIO/shooter/targetDistance", targetDistance);
    }

    // used for shoot on the move
    public static void updateIterative(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, int iterations) {
        targetDistance = calculateRobotToTargetDistance(robotPose, targetPose);
        shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
        Time timeOfFlight = calculateTimeOfFlight(calculateFuelExitVelocity(shooterVelocity), FuelConstants.HOOD_ANGLE);

        Pose2d targetFuturePose = new Pose2d();
        for (int i = 0; i < iterations; i++) {
            targetFuturePose = calculateTargetFuturePose(targetPose, robotFieldRelativeSpeeds, timeOfFlight);
            
            // update values for new future pose
            targetDistance = calculateRobotToTargetDistance(robotPose, targetFuturePose);
            shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
            timeOfFlight = calculateTimeOfFlight(calculateFuelExitVelocity(shooterVelocity), FuelConstants.HOOD_ANGLE);
        }

        robotRotation = calculateRobotRotationToTarget(robotPose, targetFuturePose);

        Logger.recordOutput("outputs/fuelIO/shooter/targetPose", targetPose);
        Logger.recordOutput("outputs/fuelIO/shooter/targetDistance", targetDistance);
        Logger.recordOutput("outputs/fuelIO/shooter/targetFuturePose", targetFuturePose);
        Logger.recordOutput("outputs/fuelIO/shooter/timeOfFlight", timeOfFlight);
    }

    public static AngularVelocity getShooterVelocity() {
        return shooterVelocity;
    }

    public static Rotation2d getRobotRotation() {
        return robotRotation;
    }

    public static Distance getTargetDistance() {
        return targetDistance;
    }

    // ————— calculators for shooter state ————— //

    private static Distance calculateRobotToTargetDistance(Pose2d robotPose, Pose2d targetPose) {
        return Meters.of(robotPose.getTranslation().minus(targetPose.getTranslation()).getNorm());
    }

    private static Rotation2d calculateRobotRotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        Translation2d targetToRobot = targetPose.getTranslation().minus(robotPose.getTranslation());
        return new Rotation2d(Math.atan2(targetToRobot.getY(), targetToRobot.getX()));
    }

    // ————— calculators for iterative shooter state ————— //

    public static LinearVelocity calculateFuelExitVelocity(AngularVelocity angularVelocity) {
        LinearVelocity linearVelocity = InchesPerSecond.of(angularVelocity.in(RadiansPerSecond) * FuelConstants.SHOOTER_WHEEL_RADIUS.in(Inches));  // multiply by shooter wheel radius
        return linearVelocity.div(2); // because backspin: https://www.chiefdelphi.com/t/determine-flywheel-velocity-for-ball-exit-velocity/394940/2
    }

    private static Time calculateTimeOfFlight(LinearVelocity fuelExitVelocity, Angle hoodAngle) {
        double shotAngle = Math.PI / 2 - hoodAngle.in(Radians); // angle between the horizontal and the fuel's velocity vector
        return Seconds.of(targetDistance.in(Meters) / (fuelExitVelocity.in(MetersPerSecond) * Math.cos(shotAngle))); // divide the x distance the fuel must travel by the velocity of the fuel in the x direction (that's what the cosine is for)
    }

    // basically what we do here is try to account for the initial velocity given by the robot to the fuel by aiming as if the target was at a different place 
    // (e.g. if the robot is moving forwards towards the target, we don't need to give the ball as much velocity from the shooter, which is analagous to shooting at a closer target)
    private static Pose2d calculateTargetFuturePose(Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, Time timeOfFlight) {
        double x = targetPose.getX() - robotFieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double y = targetPose.getY() - robotFieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Pose2d(x, y, new Rotation2d());
    }
}