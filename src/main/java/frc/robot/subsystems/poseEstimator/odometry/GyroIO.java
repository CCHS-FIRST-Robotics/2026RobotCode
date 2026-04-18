/**
 * Original source: https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/ce187a9d0ac6341f361703cf2b24c2f41448e400/src/main/java/frc/robot/subsystems/drive/GyroIO.java
 * 
 * Changes made: 
 *  - replaced frc.robot.subsystems.drive with frc.robot.subsystems.poseEstimator.odometry;
 */

package frc.robot.subsystems.poseEstimator.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}