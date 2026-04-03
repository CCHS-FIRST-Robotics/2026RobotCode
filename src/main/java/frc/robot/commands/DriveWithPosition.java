package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.Constants.FieldConstants;

public class DriveWithPosition extends Command {
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    
    private Pose2d targetPose;
    private Transform2d targetTransform;

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Pose2d targetPose,
        boolean useAllianceFlipping
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.targetPose = targetPose;

        if (useAllianceFlipping && DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
            this.targetPose = new Pose2d(
                FieldConstants.FIELD_WIDTH_X.minus(targetPose.getMeasureX()), 
                FieldConstants.FIELD_WIDTH_Y.minus(targetPose.getMeasureY()), 
                targetPose.getRotation().plus(new Rotation2d(Degrees.of(180)))
            );
        }
    }

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Transform2d targetTransform
    ) {
        addRequirements(drive);

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.targetTransform = targetTransform;
    }

    @Override
    public void initialize() {
        if (targetTransform != null) {
            targetPose = poseEstimator.getPose().plus(targetTransform);
        }
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(poseEstimator.getPose().getX() - targetPose.getX()) < 0.05
            && Math.abs(poseEstimator.getPose().getY() - targetPose.getY()) < 0.05
            && Math.abs(poseEstimator.getPose().getRotation().getRotations() - targetPose.getRotation().getRotations()) < 0.05
            && drive.getFieldRelativeSpeeds().vxMetersPerSecond < 0.1
            && drive.getFieldRelativeSpeeds().vyMetersPerSecond < 0.1
            && drive.getFieldRelativeSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}