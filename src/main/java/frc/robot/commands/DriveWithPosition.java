package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;

public class DriveWithPosition extends Command {
    // subsystems
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    
    private Pose2d targetPose;
    private Transform2d targetTransform;

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Pose2d targetPose
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.targetPose = targetPose;
    }

    public DriveWithPosition(
        Drive drive,
        PoseEstimator poseEstimator,
        Transform2d targetTransform
    ) {
        addRequirements(drive);
        addRequirements(poseEstimator);

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.targetTransform = targetTransform;
    }

    // ! add driving to a specific apriltag

    @Override
    public void initialize() {
        if(targetTransform != null){
            targetPose = poseEstimator.getPose().plus(targetTransform);
        }
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(poseEstimator.getPose().getX() - targetPose.getX()) < 0.01
            && Math.abs(poseEstimator.getPose().getY() - targetPose.getY()) < 0.01
            && Math.abs(
                MathUtil.inputModulus(poseEstimator.getPose().getRotation().getRotations(), 0, 1)
                - MathUtil.inputModulus(targetPose.getRotation().getRotations(), 0, 1)
            ) < 0.005;
            // ! the below would probably be good to add, more testing is necessary
            // && drive.getChassisSpeeds().vxMetersPerSecond < 0.1
            // && drive.getChassisSpeeds().vyMetersPerSecond < 0.1
            // && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}