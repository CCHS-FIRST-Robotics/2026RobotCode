/**
 * Original source: https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/ce187a9d0ac6341f361703cf2b24c2f41448e400/src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java
 * 
 * Changes made: 
 *  - replaced frc.robot.subsystems.drive with frc.robot.subsystems.poseEstimator.odometry;
 *  - replaced import frc.robot.generated.TunerConstants; with import frc.robot.subsystems.drive.DriveConstants;
 *  - replaced new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.DrivetrainConstants.CANBusName) with new Pigeon2(DriveConstants.DRIVETRAIN_CONSTANTS.Pigeon2Id, DriveConstants.CAN_BUS) 
 *  - replaced Drive.ODOMETRY_FREQUENCY with DriveConstants.ODOMETRY_FREQUENCY
 */

package frc.robot.subsystems.poseEstimator.odometry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(DriveConstants.DRIVETRAIN_CONSTANTS.Pigeon2Id, DriveConstants.CAN_BUS);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
                yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}