// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants.FieldConstants;

public class ShooterCalculator {
    public static final InterpolatingTreeMap<Double, ShooterState> SHOOTER_STATE_MAP = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), // ! idk why it's inverse
        ShooterState::interpolate
    ); // ! should probably be in a constants file

    public static ShooterState getShooterStateFromMap(Pose2d robotPose, Pose2d targetPose) {
        Transform2d robotToTarget = robotPose.minus(targetPose);
        ShooterState shot = SHOOTER_STATE_MAP.get(robotToTarget.getTranslation().getNorm());
        return new ShooterState(shot.velocity, shot.angle);
    }

    public static Rotation2d getRobotRotation(Pose2d robotPose, Pose2d targetPose) {
        Transform2d robotToTarget = robotPose.minus(targetPose);
        return robotToTarget.getRotation(); // ! no no no absolutely not thi
    }

    // add getShooterStateFromMath

    public static Pose2d getTargetPoseFromRobotPosition(Pose2d robotPose) {
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