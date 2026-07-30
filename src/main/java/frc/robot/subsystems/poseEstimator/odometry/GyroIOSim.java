/**
 * Based on https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/ce187a9d0ac6341f361703cf2b24c2f41448e400/src/main/java/frc/robot/subsystems/drive/GyroIOSim.java
 */

package frc.robot.subsystems.poseEstimator.odometry;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = gyroSimulation.getGyroReading();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(
                gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

        inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    }
}