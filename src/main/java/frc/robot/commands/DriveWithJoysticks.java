package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import frc.robot.Constants;
import frc.robot.subsystems.drive.*;

public class DriveWithJoysticks extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier oSupplier;
    
    // local constants
    private static final double DEADBAND = 0.1;
    private static final double EXPONENT = 2;

    public DriveWithJoysticks(
        Drive drive, 
        DoubleSupplier xSupplier, 
        DoubleSupplier ySupplier, 
        DoubleSupplier oSupplier
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.oSupplier = oSupplier;
    }

    @Override
    public void execute() { // ! for some reason, the discretize stuff that's supposed to keep it in a straight line doesn't seem to work in Sim
        // get linear velocity vector
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

        // get angular velocity scalar
        double angularVelocity = MathUtil.applyDeadband(oSupplier.getAsDouble(), DEADBAND); // apply deadband
        angularVelocity = Math.copySign(Math.pow(angularVelocity, EXPONENT), angularVelocity); // apply exponent

        // convert to chassisSpeeds
        ChassisSpeeds speeds = new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            angularVelocity * drive.getMaxAngularSpeedRadPerSec()
        );
        
        // ! not sure if we need flipping (we don't if we zero the gyro at the start of every match)

        // run velocity
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                Constants.USE_ALLIANCE_FLIPPING ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()
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
        linearMagnitude *= linearMagnitude;

        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }
}