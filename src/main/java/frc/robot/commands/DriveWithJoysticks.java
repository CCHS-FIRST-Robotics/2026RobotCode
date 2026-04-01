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
    private final boolean useTrenchAlign;
    private final boolean xLockWhileStationary;

    private final double EXPONENT = 2;

    private boolean inTrench = false;

    public DriveWithJoysticks(
        Drive drive, 
        PoseEstimator poseEstimator,
        DoubleSupplier xVelocitySupplier, 
        DoubleSupplier yVelocitySupplier, 
        DoubleSupplier thetaVelocitySupplier,
        Supplier<Rotation2d> thetaSupplier, 
        boolean useTrenchAlign,
        boolean xLockWhileStationary
    ) {
        addRequirements(drive);

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.xVelocitySupplier = xVelocitySupplier;
        this.yVelocitySupplier = yVelocitySupplier;
        this.thetaVelocitySupplier = thetaVelocitySupplier;

        this.thetaSupplier = thetaSupplier;
        this.useTrenchAlign = useTrenchAlign;
        this.xLockWhileStationary = xLockWhileStationary;
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
            linearVelocity.getX() * DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond),
            -linearVelocity.getY() * DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond), // chassisspeeds is flipped
            -angularVelocity * DriveConstants.ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond) // chassisspeeds is flipped
        );        

        // override with supplied theta
        if (thetaSupplier != null && thetaSupplier.get() != null) {
            speeds = new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                drive.getThetaPositionController().calculate(
                    poseEstimator.getPose().getRotation().getRadians(),
                    thetaSupplier.get().getRadians()
                )
            );
        }

        // trench
        if (Constants.ENABLE_TRENCH_ALIGN && useTrenchAlign) {
            // logic for which trench zones to use
            if (Zones.TRENCH_ZONES_DEFAULT.contains(poseEstimator.getPose())) { // if we're in the default
                if (!inTrench) {
                    if (Zones.TRENCH_ZONES_ALLIANCE.contains(poseEstimator.getPose())) { // figure out what side we came in from
                        Zones.TRENCH_ZONES = Zones.TRENCH_ZONES_ALLIANCE;
                        inTrench = true;
                    } else if (Zones.TRENCH_ZONES_NEUTRAL.contains(poseEstimator.getPose())) {
                        Zones.TRENCH_ZONES = Zones.TRENCH_ZONES_NEUTRAL;
                        inTrench = true;
                    }
                }
            } else {
                Zones.TRENCH_ZONES = Zones.TRENCH_ZONES_DEFAULT;
                inTrench = false;
            }

            // override with under trench angle
            if (Zones.TRENCH_ZONES.contains(poseEstimator.getPose())) {
                double yOutput = 0;
                
                if (poseEstimator.getPose().getY() > FieldConstants.FIELD_WIDTH_Y.div(2).in(Meters)) { // top
                    yOutput = drive.getYPositionController().calculate(
                        poseEstimator.getPose().getY(),
                        FieldConstants.FIELD_WIDTH_Y.minus(FieldConstants.TRENCH_WIDTH_Y.div(2)).in(Meters)
                    );
                } else { // bottom
                    yOutput = drive.getYPositionController().calculate(
                        poseEstimator.getPose().getY(),
                        FieldConstants.TRENCH_WIDTH_Y.div(2).in(Meters)
                    );
                }

                // flip y output for red alliance because field-relative transform will flip it again
                if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                    yOutput = -yOutput;
                }

                double robotYawRadians = poseEstimator.getPose().getRotation().getRadians();
                speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    yOutput,
                    drive.getThetaPositionController().calculate(
                        robotYawRadians,
                        Math.abs(robotYawRadians) < Math.PI / 2 ? 0 : Math.PI
                    )
                );
            }
        }

        // override with x lock
        if (xLockWhileStationary
            && Math.abs(speeds.vxMetersPerSecond) < 0.05
            && Math.abs(speeds.vyMetersPerSecond) < 0.05
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.05
        ) {
            drive.xLock();
            return;
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