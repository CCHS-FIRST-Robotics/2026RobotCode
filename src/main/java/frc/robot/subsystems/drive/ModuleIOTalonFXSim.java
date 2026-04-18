/**
 * Original source: https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/ce187a9d0ac6341f361703cf2b24c2f41448e400/src/main/java/frc/robot/subsystems/drive/ModuleIOTalonFXSim.java
 * 
 * Changes made: 
 *  - replaced import frc.robot.util.PhoenixUtil; with import frc.robot.utils.PhoenixUtil;
 *  - added @SuppressWarnings({ "rawtypes", "unchecked" }) above line 30
 */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.utils.PhoenixUtil;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module constants from Phoenix.
 * Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
    private final SwerveModuleSimulation simulation;

    @SuppressWarnings({ "rawtypes", "unchecked" })
    public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
        super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

        this.simulation = simulation;
        simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

        simulation.useSteerMotorController(
                new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        super.updateInputs(inputs);

        // Update odometry inputs
        inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

        inputs.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();

        inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
    }
}