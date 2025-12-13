package frc.robot.subsystems.poseEstimator;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;

public class PoseEstimator extends SubsystemBase implements Odometry.OdometryConsumer, Vision.VisionConsumer {
    private final Odometry odometry;
    private final Vision vision;

    private final SwerveDrivePoseEstimator odometryEstimator;
    private final SwerveDrivePoseEstimator visionEstimator;
    private final SwerveDrivePoseEstimator combinedEstimator;

    private final Drive drive;

    public PoseEstimator(
        GyroIO gyroIO, 
        CameraIO[] cameraIOs, 
        Drive drive, 
        Pose2d startPose
    ) {
        odometry = new Odometry(gyroIO, this, drive);
        vision = new Vision(cameraIOs, this, drive);

        odometryEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, 
            odometry.getYaw(), 
            drive.getModulePositions(),
            startPose
        );
        visionEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, 
            odometry.getYaw(), 
            drive.getModulePositions(),
            startPose,
            VecBuilder.fill(Integer.MAX_VALUE, Integer.MAX_VALUE, Integer.MAX_VALUE), // makes it not trust odometry info at all
            VecBuilder.fill(0.4, 0.4, 0.2) // is overridden anyways
        );
        combinedEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS, 
            odometry.getYaw(), 
            drive.getModulePositions(),
            startPose
        );

        this.drive = drive;

        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true); // https://github.com/PhotonVision/photonvision/pull/1662
    }

    @Override
    public void periodic() {  
        odometry.periodic();
        vision.periodic();
        
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/odometryPoseEstimate", odometryEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/visionPoseEstimate", visionEstimator.getEstimatedPosition());
        Logger.recordOutput("outputs/poseEstimator/poseEstimates/combinedPoseEstimate", combinedEstimator.getEstimatedPosition());
    }

    public void resetPosition(Pose2d pose) { // ! I don't remember if or why this works
        // "the library automatically takes care of offsetting the gyro angle" - SwerveDrivePoseEstimator.resetPosition
        odometryEstimator.resetPosition(odometry.getYaw(), drive.getModulePositions(), pose);
        visionEstimator.resetPosition(odometry.getYaw(), drive.getModulePositions(), pose);
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
        return visionEstimator.getEstimatedPosition();
    }

    // @SuppressWarnings("unused")
    private Pose2d getCombinedPose() {
        return combinedEstimator.getEstimatedPosition();
    }

    @Override 
    public void acceptOdometry(
        int sampleCount, 
        double[] sampleTimestamps, 
        Rotation2d[] sampleGyroYaws, 
        SwerveModulePosition[][] sampleModulePositions
    ) {
        for (int i = 0; i < sampleCount; i++) {
            // odometry
            odometryEstimator.updateWithTime(
                sampleTimestamps[i],
                sampleGyroYaws[i],
                sampleModulePositions[i]
            );
            // combined
            combinedEstimator.updateWithTime(
                sampleTimestamps[i],
                sampleGyroYaws[i],
                sampleModulePositions[i]
            );
        }

        // vision (blank because odom is ignored)
        visionEstimator.updateWithTime(
            Timer.getFPGATimestamp(),
            new Rotation2d(),
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            }
        );
    }

    @Override
    public void acceptVision(
        Pose2d visionRobotPoseMeters, 
        double timestampSeconds, 
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        visionEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        combinedEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}