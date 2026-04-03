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
    private boolean useAllianceFlipping = false;

    private Transform2d targetTransform;

    private Pose2d calculatedTargetPose;

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

        this.useAllianceFlipping = useAllianceFlipping;
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
        if (!useAllianceFlipping) {
            calculatedTargetPose = targetPose;
        } else {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) != Alliance.Blue) {
                calculatedTargetPose = new Pose2d(
                    FieldConstants.FIELD_WIDTH_X.minus(targetPose.getMeasureX()), 
                    FieldConstants.FIELD_WIDTH_Y.minus(targetPose.getMeasureY()), 
                    targetPose.getRotation().plus(new Rotation2d(Degrees.of(180)))
                );
            } else {
                calculatedTargetPose = targetPose;
            }
        }

        if (targetTransform != null) {
            calculatedTargetPose = poseEstimator.getPose().plus(targetTransform);
        }
    }

    @Override
    public void execute() {
        drive.runPosition(calculatedTargetPose);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(poseEstimator.getPose().getX() - calculatedTargetPose.getX()) < 0.05
            && Math.abs(poseEstimator.getPose().getY() - calculatedTargetPose.getY()) < 0.05
            && Math.abs(poseEstimator.getPose().getRotation().getRotations() - calculatedTargetPose.getRotation().getRotations()) < 0.05
            && drive.getFieldRelativeSpeeds().vxMetersPerSecond < 0.1
            && drive.getFieldRelativeSpeeds().vyMetersPerSecond < 0.1
            && drive.getFieldRelativeSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}