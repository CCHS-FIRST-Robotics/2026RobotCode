package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;

public class PoseEstimator extends SubsystemBase {
    private final Odometry odometry;
    private final Vision vision;

    private final SwerveDrivePoseEstimator odometryEstimator;
    private Pose2d visionEstimate;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private final Drive drive;

    public PoseEstimator(
        GyroIO gyroIO, 
        CameraIO[] cameraIOs, 
        Drive drive
    ) {
        odometry = new Odometry(gyroIO, drive);
        vision = new Vision(cameraIOs);

        odometryEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(),
            new Pose2d()
        );
        visionEstimate = new Pose2d();
        combinedEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, 
            new Rotation2d(), 
            drive.getModulePositions(),
            new Pose2d()
        );

        this.drive = drive;

        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true); // https://github.com/PhotonVision/photonvision/pull/1662
    }

    @Override
    public void periodic() {  
        odometry.periodic();
        vision.periodic();
        
        // odometry
        int odometrySampleCount = odometry.getSampleCount();
        for(int i = 0; i < odometrySampleCount; i++){
            odometryEstimator.updateWithTime(
                odometry.getSampleTimestamps()[i],
                odometry.getSampleGyroYaws()[i],
                odometry.getSampleModulePositions()[i]
            );
        }
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());

        // vision
        visionEstimate = vision.getVisionEstimate(); // based on the last time it saw an apriltag
        for(int i = 0; i < odometrySampleCount; i++){
            combinedEstimator.updateWithTime(
                odometry.getSampleTimestamps()[i],
                odometry.getSampleGyroYaws()[i],
                odometry.getSampleModulePositions()[i]
            );
        }
        combinedEstimator.addVisionMeasurement(visionEstimate, vision.getLatestTimeStamp());
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/visionPoseEstimate", visionEstimate);
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public void resetPosition(Pose2d pose) { // ! I don't remember if or why this works
        // "the library automatically takes care of offsetting the gyro angle" - SwerveDrivePoseEstimator.resetPosition
        odometryEstimator.resetPosition(odometry.getYaw(), drive.getModulePositions(), pose);
        visionEstimate = new Pose2d();
        combinedEstimator.resetPosition(odometry.getYaw(), drive.getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return getCombinedPose();
    }

    @SuppressWarnings("unused")
    private Pose2d getOdometryPose() {
        return odometryEstimator.getEstimatedPosition();
    }

    @SuppressWarnings("unused")
    private Pose2d getVisionPose() {
        return visionEstimate;
    }

    // @SuppressWarnings("unused")
    private Pose2d getCombinedPose() {
        return combinedEstimator.getEstimatedPosition();
    }
}