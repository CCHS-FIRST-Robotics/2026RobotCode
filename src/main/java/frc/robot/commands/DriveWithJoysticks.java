package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.*;
import java.util.function.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Zones;

public class DriveWithJoysticks extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
 
    private final DoubleSupplier xVelocitySupplier;
    private final DoubleSupplier yVelocitySupplier;
    private final DoubleSupplier thetaVelocitySupplier;

    private final Supplier<Rotation2d> thetaSupplier;

    private final double EXPONENT = 2;

    public DriveWithJoysticks(
        Drive drive, 
        PoseEstimator poseEstimator,
        DoubleSupplier xVelocitySupplier, 
        DoubleSupplier yVelocitySupplier, 
        DoubleSupplier thetaVelocitySupplier,
        Supplier<Rotation2d> thetaSupplier
    ) {
        addRequirements(drive);

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.thetaVelocitySupplier = thetaVelocitySupplier;

        this.thetaSupplier = thetaSupplier;
    }

    @Override
    public void execute() {
        // get linear velocity vector
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xVelocitySupplier.getAsDouble(), yVelocitySupplier.getAsDouble());

        // get angular velocity scalar
        double angularVelocity = thetaVelocitySupplier.getAsDouble();
        angularVelocity = Math.copySign(Math.pow(angularVelocity, EXPONENT), angularVelocity); // apply exponent

        // convert to chassisSpeeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond),
            -linearVelocity.getY() * DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond), // chassisspeeds is flipped
            -angularVelocity * DriveConstants.MAX_ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond) // chassisspeeds is flipped
        );

        // override with supplied theta
        if (thetaSupplier != null) {
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                drive.getThetaController().calculate(
                    poseEstimator.getPose().getRotation().getRadians(),
                    thetaSupplier.get().getRadians()
                )
            );
        }

        // override with under trench angle
        if (Constants.TRENCH_ALIGN && Zones.TRENCH_ZONES.contains(poseEstimator.getPose())) {
            double yOutput = 0;
            if (poseEstimator.getPose().getY() > FieldConstants.FIELD_WIDTH_Y.div(2).in(Meters)) { // top
                yOutput = drive.getYController().calculate(
                    poseEstimator.getPose().getY(),
                    FieldConstants.FIELD_WIDTH_Y.minus(FieldConstants.TRENCH_WIDTH_Y.div(2)).in(Meters)
                );
            } else { // bottom
                yOutput = drive.getYController().calculate(
                    poseEstimator.getPose().getY(),
                    FieldConstants.TRENCH_WIDTH_Y.div(2).in(Meters)
                );
            }

            double robotYawRadians = poseEstimator.getPose().getRotation().getRadians();
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                yOutput,
                drive.getThetaController().calculate(
                    robotYawRadians,
                    Math.abs(robotYawRadians) < Math.PI / 2 ? 0 : Math.PI
                )
            );
        }
                
        // run velocity
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
                poseEstimator.getPose().getRotation() : 
                poseEstimator.getPose().getRotation().plus(new Rotation2d(Math.PI)) // flip if red alliance
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = Math.pow(Math.hypot(x, y), EXPONENT); // apply exponent
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}