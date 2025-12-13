package frc.robot.subsystems.poseEstimator.odometry;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class Odometry {
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();
    
    private final OdometryConsumer consumer;
    
    private final Drive drive;
    
    public static final Lock odometryLock = new ReentrantLock();
    
    private int sampleCount;
    private double[] sampleTimestamps = new double[0];
    private Rotation2d[] sampleGyroYaws = new Rotation2d[0];
    private SwerveModulePosition[][] sampleModulePositions = new SwerveModulePosition[][] {
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()
        }
    };
    private SwerveModulePosition[][] sampleModuleDeltas = new SwerveModulePosition[][] {
        new SwerveModulePosition[] {
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()
        }
    };

    private Rotation2d gyroYaw = new Rotation2d();

    public Odometry(GyroIO gyroIO, OdometryConsumer consumer, Drive drive){
        this.gyroIO = gyroIO;
        this.consumer = consumer;
        this.drive = drive;

        PhoenixOdometryThread.getInstance().start(); // start odometry thread
    }
    
    public void periodic() { // https://v6.docs.ctr-electronics.com/en/latest/docs/application-notes/update-frequency-impact.html
        odometryLock.lock(); // prevents odometry updates while reading data
        gyroIO.updateInputs(inputs);
        Logger.processInputs("poseEstimator/gyro", inputs);
        drive.modulePeriodic(); // run module.periodic for each swerve module
        odometryLock.unlock();
        
        drive.updateModuleSamples();

        sampleCount = drive.getSampleCount();
        
        sampleTimestamps = drive.getSampleTimestamps();
        
        sampleGyroYaws = new Rotation2d[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            if (inputs.connected) { // use real gyro angle
                gyroYaw = inputs.odometryYawPositions[i];
            } else { // use inverse kinematics to estimate angle
                Twist2d twist = DriveConstants.KINEMATICS.toTwist2d(sampleModuleDeltas[i]);
                gyroYaw = gyroYaw.plus(new Rotation2d(twist.dtheta));
            }
            sampleGyroYaws[i] = gyroYaw;
        }
        
        sampleModulePositions = drive.getSampleModulePositions();
        
        sampleModuleDeltas = drive.getSampleModuleDeltas();

        consumer.acceptOdometry(sampleCount, sampleTimestamps, sampleGyroYaws, sampleModulePositions);
    }

    public Rotation2d getYaw() {
        return gyroYaw;
    }

    @FunctionalInterface
    public interface OdometryConsumer {
        void acceptOdometry(
            int sampleCount, 
            double[] sampleTimestamps, 
            Rotation2d[] sampleGyroYaws, 
            SwerveModulePosition[][] sampleModulePositions
        );
    }
}