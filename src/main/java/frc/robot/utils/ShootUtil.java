// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.subsystems.fuelIO.FuelConstants;

public class ShootUtil {
    public static final InterpolatingTreeMap<Double, ShooterState> SHOOTER_STATE_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        ShooterState::interpolate
    );

    /**
     * SHOOTER_STATE_MAP.put(5.560, new ShooterState(RotationsPerSecond.of(68.203125), Rotations.of(0.077392578125)));
     * SHOOTER_STATE_MAP.put(5.065, new ShooterState(RotationsPerSecond.of(60.70703125), Rotations.of(0.06127929687500001)));
     * SHOOTER_STATE_MAP.put(4.538, new ShooterState(RotationsPerSecond.of(60.3671875), Rotations.of(0.06127929687500001)));
     * SHOOTER_STATE_MAP.put(4.024, new ShooterState(RotationsPerSecond.of(55.40234375), Rotations.of(0.0517578125)));
     * SHOOTER_STATE_MAP.put(3.449, new ShooterState(RotationsPerSecond.of(52.06054687499999), Rotations.of(0.0517578125)));
     * 
     * 
     * 
     * 
     */

    static {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL) {
            SHOOTER_STATE_MAP.put(5.227799, new ShooterState(RotationsPerSecond.of(61.33203125), Rotations.of(0.0517578125)));
            SHOOTER_STATE_MAP.put(3.432713, new ShooterState(RotationsPerSecond.of(52.4296875), Rotations.of(0.037353515625)));
            SHOOTER_STATE_MAP.put(1.811582, new ShooterState(RotationsPerSecond.of(45.8046875), Rotations.of(0.002197265625)));


            // ! measured with kicker at 3 volts
            // SHOOTER_STATE_MAP.put(5.892394, new ShooterState(RotationsPerSecond.of(69.212891), Rotations.of(0.051758)));
            // SHOOTER_STATE_MAP.put(4.077568, new ShooterState(RotationsPerSecond.of(58.449219), Rotations.of(0.051758)));
            // SHOOTER_STATE_MAP.put(3.172064, new ShooterState(RotationsPerSecond.of(49.283203), Rotations.of(0.031982)));
            // SHOOTER_STATE_MAP.put(2.18186, new ShooterState(RotationsPerSecond.of(46.9375), Rotations.of(0.002197)));
        } else {
            SHOOTER_STATE_MAP.put(1.6601040952847508, new ShooterState(RotationsPerSecond.of(37.734372), Rotations.of(0.04)));
            SHOOTER_STATE_MAP.put(1.9083062499099746, new ShooterState(RotationsPerSecond.of(45.281246), Rotations.of(0.03)));
            SHOOTER_STATE_MAP.put(2.597315571788894, new ShooterState(RotationsPerSecond.of(45.281246), Rotations.of(0.04)));
            SHOOTER_STATE_MAP.put(3.134703804104257, new ShooterState(RotationsPerSecond.of(50.312496), Rotations.of(0.04)));
            SHOOTER_STATE_MAP.put(3.7132919261998163, new ShooterState(RotationsPerSecond.of(47.796872), Rotations.of(0.06)));
            SHOOTER_STATE_MAP.put(4.3661181434947265, new ShooterState(RotationsPerSecond.of(57.85937), Rotations.of(0.04)));
            SHOOTER_STATE_MAP.put(4.439996114327355, new ShooterState(RotationsPerSecond.of(47.796872), Rotations.of(0.08)));
        }
    }

    private static ShooterState shooterState = new ShooterState();
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
        shooterState = SHOOTER_STATE_MAP.get(targetDistance.in(Meters));
        robotRotation = calculateRobotRotationToTarget(robotPose, targetPose);

        Logger.recordOutput("outputs/fuelIO/shooter/targetPose", targetPose);
        Logger.recordOutput("outputs/fuelIO/shooter/targetDistance", targetDistance);
    }

    public static void updateIterative(Pose2d robotPose, Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, int iterations) {
        Distance targetDistance = calculateRobotToTargetDistance(robotPose, targetPose);
        shooterState = SHOOTER_STATE_MAP.get(targetDistance.in(Meters));
        Time timeOfFlight = calculateTimeOfFlight(calculateShooterLinearVelocity(shooterState.velocity), shooterState.angle, targetDistance);

        Pose2d targetFuturePose = new Pose2d();
        for(int i = 0; i < iterations; i++) {
            targetFuturePose = calculateTargetFuturePose(targetPose, robotFieldRelativeSpeeds, timeOfFlight); // move the target as much as the robot would move in timeOfFlight seconds
            
            // update values for new future pose
            targetDistance = calculateRobotToTargetDistance(robotPose, targetFuturePose);
            shooterState = SHOOTER_STATE_MAP.get(targetDistance.in(Meters));
            timeOfFlight = calculateTimeOfFlight(calculateShooterLinearVelocity(shooterState.velocity), shooterState.angle, targetDistance);
        }

        robotRotation = calculateRobotRotationToTarget(robotPose, targetFuturePose);

        Logger.recordOutput("outputs/fuelIO/shooter/targetPose", targetPose);
        Logger.recordOutput("outputs/fuelIO/shooter/targetDistance", targetDistance);
        Logger.recordOutput("outputs/fuelIO/shooter/targetFuturePose", targetFuturePose);
        Logger.recordOutput("outputs/fuelIO/shooter/timeOfFlight", timeOfFlight);
    }

    public static ShooterState getShooterState() {
        return shooterState;
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

    private static Time calculateTimeOfFlight(LinearVelocity velocity, Angle angle, Distance distance) {
        double hoodShotAngle = Math.PI / 2 - angle.in(Radians);
        return Seconds.of(distance.in(Meters) / (velocity.in(MetersPerSecond) * Math.cos(hoodShotAngle))); // only accounts for x direction (that's what the cosine is for)
    }

    private static Pose2d calculateTargetFuturePose(Pose2d targetPose, ChassisSpeeds robotFieldRelativeSpeeds, Time timeOfFlight) {
        double x = targetPose.getX() - robotFieldRelativeSpeeds.vxMetersPerSecond * timeOfFlight.in(Seconds);
        double y = targetPose.getY() - robotFieldRelativeSpeeds.vyMetersPerSecond * timeOfFlight.in(Seconds);

        return new Pose2d(x, y, new Rotation2d());
    }

    public static record ShooterState(
        AngularVelocity velocity, 
        Angle angle
    ) {
        public ShooterState() {
            this(RotationsPerSecond.of(0), Rotations.of(0));
        }

        public static ShooterState interpolate(ShooterState start, ShooterState end, double t) {
            return new ShooterState(
                RotationsPerSecond.of(MathUtil.interpolate(
                    start.velocity.in(RotationsPerSecond), 
                    end.velocity.in(RotationsPerSecond), 
                    t
                )),
                Rotations.of(MathUtil.interpolate(
                    start.angle.in(Rotations), 
                    end.angle.in(Rotations), 
                t))
            );
        }
    }
}