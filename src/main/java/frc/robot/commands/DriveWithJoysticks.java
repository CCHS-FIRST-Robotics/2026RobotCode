package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.poseEstimator.PoseEstimator;

public class DriveWithJoysticks extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier oSupplier;
    
    private final double DEADBAND = 0.1;
    private final double EXPONENT = 2;

    public DriveWithJoysticks(
        Drive drive, 
        PoseEstimator poseEstimator,
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier oSupplier
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.oSupplier = oSupplier;
    }

    @Override
    public void execute() {
        // get linear velocity vector
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // get angular velocity scalar
        double angularVelocity = MathUtil.applyDeadband(oSupplier.getAsDouble(), DEADBAND); // apply deadband
        angularVelocity = Math.copySign(Math.pow(angularVelocity, EXPONENT), angularVelocity); // apply exponent

        // convert to chassisSpeeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond),
            -linearVelocity.getY() * DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond), // chassisspeeds is flipped
            -angularVelocity * DriveConstants.MAX_ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond) // chassisspeeds is flipped
        );
        
        // run velocity
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Constants.USE_ALLIANCE_FLIPPING ? poseEstimator.getPose().getRotation().plus(new Rotation2d(Math.PI)) : poseEstimator.getPose().getRotation()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // apply exponent
        linearMagnitude = Math.pow(linearMagnitude, EXPONENT);

        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}