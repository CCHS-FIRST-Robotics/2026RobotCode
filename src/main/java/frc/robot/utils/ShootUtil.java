// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    /**
     */

    static {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            SHOOTER_VELOCITY_MAP.put(0.0, 0.0); // ! 
        } else {
            SHOOTER_VELOCITY_MAP.put(1.908, 46.539058922493794);
            SHOOTER_VELOCITY_MAP.put(2.998, 57.8593705522896);
            SHOOTER_VELOCITY_MAP.put(3.919, 65.40624497215343);
        }
    }

    private static AngularVelocity shooterVelocity = RotationsPerSecond.of(0);
    private static Rotation2d robotRotation = new Rotation2d();

    // ————— public functions ————— //

    public static Pose2d getTargetPose(Pose2d robotPose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) { // flip everything to blue alliance reference frame
            robotPose = FieldConstants.calculateAllianceFlippedPose(robotPose);
        }

        Pose2d targetPose = new Pose2d();

        if (robotPose.getX() < FieldConstants.ALLIANCE_ZONE_WIDTH_X.in(Meters)) { // hub
            targetPose = FieldConstants.BLUE_HUB.toPose2d();
        } else { // passing
            if (robotPose.getY() > FieldConstants.FIELD_WIDTH_Y.div(2).in(Meters)) { // left
                targetPose = FieldConstants.BLUE_PASS_LEFT;
            } else {
                targetPose = FieldConstants.BLUE_PASS_RIGHT;
            }
        }

        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return targetPose;
        } else {
            return FieldConstants.calculateAllianceFlippedPose(targetPose); // rotate around center for red alliance
        }
    }
    
    public static void update(Pose2d robotPose, Pose2d targetPose) {
        Distance targetDistance = calculateRobotToTargetDistance(robotPose, targetPose);
        shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
        robotRotation = calculateRobotRotationToTarget(robotPose, targetPose);

        Logger.recordOutput("outputs/fuelIO/shooter/targetPose", targetPose);
        Logger.recordOutput("outputs/fuelIO/shooter/targetDistance", targetDistance);
    }

    public static void updateIterative(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, int iterations) {
        Distance targetDistance = calculateRobotToTargetDistance(robotPose, targetPose);
        shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
        Time timeOfFlight = calculateTimeOfFlight(calculateShooterLinearVelocity(shooterVelocity), FuelConstants.HOOD_ANGLE, targetDistance);

        Pose2d targetFuturePose = new Pose2d();
        for(int i = 0; i < iterations; i++) {
            targetFuturePose = calculateTargetFuturePose(targetPose, robotFieldRelativeSpeeds, timeOfFlight); // move the target as much as the robot would move in timeOfFlight seconds
            
            // update values for new future pose
            targetDistance = calculateRobotToTargetDistance(robotPose, targetFuturePose);
            shooterVelocity = RotationsPerSecond.of(SHOOTER_VELOCITY_MAP.get(targetDistance.in(Meters)));
            timeOfFlight = calculateTimeOfFlight(calculateShooterLinearVelocity(shooterVelocity), FuelConstants.HOOD_ANGLE, targetDistance);
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

    // ————— calculators for shooter state ————— //

    private static Distance calculateRobotToTargetDistance(Pose2d robotPose, Pose2d targetPose) {
        return Meters.of(robotPose.getTranslation().minus(targetPose.getTranslation()).getNorm());
    }

    private static Rotation2d calculateRobotRotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        Translation2d targetToRobot = targetPose.getTranslation().minus(robotPose.getTranslation());
        return new Rotation2d(Math.atan2(targetToRobot.getY(), targetToRobot.getX()));
    }

    // ————— calculators for iterative shooter state ————— //

    public static LinearVelocity calculateShooterLinearVelocity(AngularVelocity angularVelocity) {
        LinearVelocity linearVelocity = InchesPerSecond.of(angularVelocity.in(RadiansPerSecond) * FuelConstants.SHOOTER_WHEEL_RADIUS.in(Inches));  // multiply by shooter wheel radius
        return linearVelocity.div(2); // because backspin: https://www.chiefdelphi.com/t/determine-flywheel-velocity-for-ball-exit-velocity/394940/2
    }

    private static Time calculateTimeOfFlight(LinearVelocity shooterVelocity, Angle hoodAngle, Distance distance) {
        double shotAngle = Math.PI / 2 - hoodAngle.in(Radians); // angle between the horizontal and the ball's velocity vector
        return Seconds.of(distance.in(Meters) / (shooterVelocity.in(MetersPerSecond) * Math.cos(shotAngle))); // only accounts for x direction (that's what the cosine is for)
    }

    private static Pose2d calculateTargetFuturePose(Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, Time timeOfFlight) {
        double x = targetPose.getX() - robotFieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double y = targetPose.getY() - robotFieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Pose2d(x, y, new Rotation2d());
    }
}