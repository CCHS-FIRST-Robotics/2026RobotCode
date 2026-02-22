// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.*;
import edu.wpi.first.math.interpolation.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import java.util.function.Supplier;
import frc.robot.Constants.*;

public class ShooterCalculator {
    public static final InterpolatingTreeMap<Double, ShooterState> SHOOTER_STATE_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(),
        ShooterState::interpolate
    );
    
    static {
        SHOOTER_STATE_MAP.put(4.3661181434947265, new ShooterState(RotationsPerSecond.of(28.929685), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(1.9083062499099746, new ShooterState(RotationsPerSecond.of(22.640623), Rotations.of(0.03)));
        SHOOTER_STATE_MAP.put(4.439996114327355, new ShooterState(RotationsPerSecond.of(23.898436), Rotations.of(0.08)));
        SHOOTER_STATE_MAP.put(1.6601040952847508, new ShooterState(RotationsPerSecond.of(18.867186), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(3.134703804104257, new ShooterState(RotationsPerSecond.of(25.156248), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(2.597315571788894, new ShooterState(RotationsPerSecond.of(22.640623), Rotations.of(0.04)));
        SHOOTER_STATE_MAP.put(3.7132919261998163, new ShooterState(RotationsPerSecond.of(23.898436), Rotations.of(0.06)));
    } // ! I wonder how they're doing it for passing

    // get the shooter state in order to shoot at the target pose
    public static ShooterState getShooterStateFromMap(Pose2d robotPose, Pose2d targetPose) {
        Transform2d robotToTarget = robotPose.minus(targetPose);
        ShooterState shot = SHOOTER_STATE_MAP.get(robotToTarget.getTranslation().getNorm());
        return new ShooterState(shot.velocity, shot.angle);
    }

    // get the field-relative angle that the robot must face in order to point at the supplied target pose
    public static Rotation2d getRobotRotationToTarget(Pose2d robotPose, Pose2d targetPose) {
        Translation2d targetToRobot = targetPose.getTranslation().minus(robotPose.getTranslation());
        return new Rotation2d(Math.atan2(targetToRobot.getY(), targetToRobot.getX()));
    }

    // add getShooterStateFromMath

    // get where the robot should be aiming based on its position on the field and its allaince
    public static Pose2d getTargetPoseFromRobotPosition(Supplier<Pose2d> robotPoseSupplier) {
        Pose2d robotPose = robotPoseSupplier.get();
        if (robotPose.getX() < FieldConstants.ALLIANCE_ZONE_WIDTH_X.in(Meters)) {
            return FieldConstants.BLUE_HUB.toPose2d();
        }

        return FieldConstants.BLUE_PASS;

        // passing logic
        // ! awwww fuck we need another case depending on alliance
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