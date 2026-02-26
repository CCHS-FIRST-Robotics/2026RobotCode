// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.*;

public class Calculator {
    public static final InterpolatingTreeMap<Double, ShooterState> SHOOTER_STATE_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        ShooterState::interpolate
    );
    
    static {
        SHOOTER_STATE_MAP.put(4.3661181434947265, new ShooterState(RotationsPerSecond.of(57.85937), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(1.9083062499099746, new ShooterState(RotationsPerSecond.of(45.281246), Rotations.of(0.03)));
        SHOOTER_STATE_MAP.put(4.439996114327355, new ShooterState(RotationsPerSecond.of(47.796872), Rotations.of(0.08)));
        SHOOTER_STATE_MAP.put(1.6601040952847508, new ShooterState(RotationsPerSecond.of(37.734372), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(3.134703804104257, new ShooterState(RotationsPerSecond.of(50.312496), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(2.597315571788894, new ShooterState(RotationsPerSecond.of(45.281246), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(3.7132919261998163, new ShooterState(RotationsPerSecond.of(47.796872), Rotations.of(0.06)));
    }

    // get the shooter state in order to shoot at the target pose
    public static ShooterState getShooterStateFromMap(double distance) {
        ShooterState shot = SHOOTER_STATE_MAP.get(distance);
        return new ShooterState(shot.velocity, shot.angle);
    }

    // get the field-relative angle that the robot must face in order to point at the supplied target pose
    public static Rotation2d getRobotRotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        Translation2d targetToRobot = targetPose.getTranslation().minus(robotPose.getTranslation());
        return new Rotation2d(Math.atan2(targetToRobot.getY(), targetToRobot.getX()));
    }

    // get where the robot should be aiming based on its position on the field and its allaince
    public static Pose2d getTargetPoseFromRobotPosition(Pose2d robotPose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) { // flip everything to blue alliance reference frame
            robotPose = getAllianceFlippedPose(robotPose);
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
            return getAllianceFlippedPose(targetPose); // rotate around center for red alliance
        }
    }

    public static Pose2d getAllianceFlippedPose(Pose2d pose) {
        return pose.rotateAround(
            new Translation2d(FieldConstants.FIELD_WIDTH_X.div(2), FieldConstants.FIELD_WIDTH_Y.div(2)), 
            new Rotation2d(Degrees.of(180))
        );
    }

    public static record ShooterState(AngularVelocity velocity, Angle angle) {
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