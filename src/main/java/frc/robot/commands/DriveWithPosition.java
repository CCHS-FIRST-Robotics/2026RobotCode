/**
 * Original code
 */

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

    /**
     * Overload for using a Pose2d (e.g. if we want the robot to drive to a set location on the field)
     * 
     * @param drive - the drive object
     * @param poseEstimator - the poseEstimator object
     * @param targetPose - the desired Pose2d
     * @param useAllianceFlipping - whether the robot should go to the absolute provided Pose2d or if it should be relative to alliance (e.g. if we want a command to drive the robot to the outpost, you just tell it the outpost position while in the blue alliance)
     */
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

    /**
     * Overload for using a Transform2d (e.g. if we want the robot to drive 2 meters backwards from wherever it is)
     * 
     * @param drive - the drive object
     * @param poseEstimator - the poseEstimator object
     * @param targetTransform - the desired Transform2d
     */
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
        // these thresholds were verified experimentally, and are necessary to make sure the command doesn't run forever
        return Math.abs(poseEstimator.getPose().getX() - calculatedTargetPose.getX()) < 0.03
            && Math.abs(poseEstimator.getPose().getY() - calculatedTargetPose.getY()) < 0.03
            && Math.abs(poseEstimator.getPose().getRotation().getRotations() - calculatedTargetPose.getRotation().getRotations()) < 0.06
            && Math.abs(drive.getFieldRelativeSpeeds().vxMetersPerSecond) < 0.03
            && Math.abs(drive.getFieldRelativeSpeeds().vyMetersPerSecond) < 0.03
            && Math.abs(drive.getFieldRelativeSpeeds().omegaRadiansPerSecond) < 0.03;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}