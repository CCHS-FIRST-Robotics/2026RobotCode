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
    
    public static final Lock odometryLock = new ReentrantLock();
    
    private final Drive drive;
    
    int sampleCount;
    double[] sampleTimestamps = new double[0];
    Rotation2d[] sampleGyroYaws = new Rotation2d[0];
    Rotation2d gyroYaw = new Rotation2d();
    SwerveModulePosition[][] sampleModulePositions = new SwerveModulePosition[][] {
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()
        }
    };
    SwerveModulePosition[][] sampleModuleDeltas = new SwerveModulePosition[][] {
        new SwerveModulePosition[] {
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition(), 
            new SwerveModulePosition()
        }
    };

    public Odometry(GyroIO gyroIO, Drive drive){
        this.gyroIO = gyroIO;
        this.drive = drive;

        PhoenixOdometryThread.getInstance().start(); // start odometry thread
    }
    
    public void periodic() {
        odometryLock.lock(); // prevents odometry updates while reading data
        gyroIO.updateInputs(inputs);
        Logger.processInputs("outputs/poseEstimator/gyro", inputs);
        drive.modulePeriodic(); // run module.periodic for each swerve module
        odometryLock.unlock();

        // ————— get all module and gyro time-synced samples ————— //
        
        drive.updateModuleSamples();

        sampleCount = drive.getSampleCount();
        
        sampleTimestamps = drive.getSampleTimestamps();
        
        sampleGyroYaws = new Rotation2d[sampleCount];
        for(int i = 0; i < sampleCount; i++){
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
    }

    public int getSampleCount() {
        return sampleCount;
    }

    public double[] getSampleTimestamps() {
        return sampleTimestamps;
    }

    public Rotation2d[] getSampleGyroYaws() {
        return sampleGyroYaws;
    }

    public SwerveModulePosition[][] getSampleModulePositions() {
        return sampleModulePositions;
    }

    public Rotation2d getYaw() {
        return gyroYaw;
    }
}