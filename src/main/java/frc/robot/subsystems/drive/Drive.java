package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import org.littletonrobotics.junction.*;
import choreo.trajectory.SwerveSample;

public class Drive extends SubsystemBase {    
    private enum DRIVE_MODE {
        DISABLED,
        POSITION,
        VELOCITY,
        CHARACTERIZING
    };
    private DRIVE_MODE controlMode = DRIVE_MODE.DISABLED;
    
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    
    // ————— odometry ————— //

    private PoseEstimator poseEstimator;

    private double[] sampleTimestamps = new double[0];
    private int sampleCount = 0;
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
    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    // ————— characterization ————— //

    private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1), // ramp rate
            Volts.of(1.75), // step voltage
            Seconds.of(4), // timeout
            (state) -> Logger.recordOutput("outputs/drive/sysIdState", state.toString()) // send the data to advantagekit
            // (state) -> SignalLogger.writeString("driveSysId", state.toString()) // send the data to SignalLogger
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> this.runCharacterization(
                new Voltage[] {volts, volts, volts, volts}, 
                new Angle[] {Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)}
            ), // characterization supplier
            null, // no log consumer needed since advantagekit records the data
            this
        )
    );
    
    private Voltage[] characterizationVolts = {
        Volts.of(0), 
        Volts.of(0), 
        Volts.of(0), 
        Volts.of(0)
    };
    private Angle[] characterizationPositions = {
        Rotations.of(0), 
        Rotations.of(0), 
        Rotations.of(0), 
        Rotations.of(0)
    };

    // ————— position ————— //

    private final PIDController xPID = new PIDController(5, 0, 0);
    private final PIDController yPID = new PIDController(5, 0, 0);
    private final PIDController oPID = new PIDController(5, 0, 0);
    
    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();

    // ————— velocity ————— //

    private ChassisSpeeds speeds = new ChassisSpeeds();

    public Drive(
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        modules[0] = new Module(flModuleIO, 0, DriveConstants.SWERVE_MODULE_CONSTANTS[0]);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.SWERVE_MODULE_CONSTANTS[1]);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.SWERVE_MODULE_CONSTANTS[2]);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.SWERVE_MODULE_CONSTANTS[3]);
        
        oPID.enableContinuousInput(-Math.PI, Math.PI); // allows position PID to turn in the correct direction
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = DRIVE_MODE.DISABLED;
        }

        // module states
        SwerveModuleState[] moduleStatesOutput = getModuleStates();
        Logger.recordOutput("outputs/drive/moduleStatesOutput", moduleStatesOutput);
        // chassisspeeds
        ChassisSpeeds speedsOutput = DriveConstants.KINEMATICS.toChassisSpeeds(moduleStatesOutput);
        Logger.recordOutput("outputs/drive/speedsOutput", speedsOutput);

        // run control mode
        switch (controlMode) {
            case DISABLED:
                // set all module voltages to 0
                for (Module module : modules) {
                    module.stop();
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case CHARACTERIZING:
                for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(characterizationVolts[i].in(Volts), new Rotation2d(characterizationPositions[i]));
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", new SwerveModuleState[] {});
                break;
            case POSITION:
                // get PIDs
                double xOutput = xPID.calculate(poseEstimator.getPose().getX(), positionSetpoint.getX());
                double yOutput = yPID.calculate(poseEstimator.getPose().getY(), positionSetpoint.getY());
                double oOutput = oPID.calculate(poseEstimator.getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());

                // create chassisspeeds object with FOC
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    twistSetpoint.dx + xOutput,
                    twistSetpoint.dy + yOutput,
                    twistSetpoint.dtheta + oOutput,
                    poseEstimator.getPose().getRotation()
                );
                // fallthrough to VELOCITY case; no break statement needed
            case VELOCITY: 
                speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD); // explaination: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
                
                Logger.recordOutput("outputs/drive/speedsInput", speeds);

                SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds); // convert speeds to module states
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_ALLOWED_LINEAR_SPEED); // renormalize wheel speeds

                // run modules
                for (int i = 0; i < 4; i++) {
                    modules[i].runSetpoint(moduleStates[i]);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", moduleStates);
                break;
        }
    }

    // ————— running control mode ————— //

    public void stop() {
        controlMode = DRIVE_MODE.DISABLED;
    }

    public void runCharacterization(Voltage[] volts, Angle[] positions) {
        controlMode = DRIVE_MODE.CHARACTERIZING;
        characterizationVolts = volts;
        characterizationPositions = positions;
    }

    public void runPosition(Pose2d pose) {
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = pose;
        twistSetpoint = new Twist2d();
    }

    public void runAutoPosition(SwerveSample sample) {
        controlMode = DRIVE_MODE.POSITION;
        positionSetpoint = sample.getPose();
        twistSetpoint = sample.getChassisSpeeds().toTwist2d(0.02);
    }

    public void runVelocity(ChassisSpeeds speedsInput) {
        controlMode = DRIVE_MODE.VELOCITY;
        speeds = speedsInput;
    }

    // ————— sysid ————— // 

    public Command sysIdFull() {
        return driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    // ————— poseEstimator ————— //

    public void setPoseEstimator(PoseEstimator poseEstimator) {
        this.poseEstimator = poseEstimator;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = modules[i].getState();
        }
        return moduleStates;
    }

    // ————— odometry ————— //

    public void modulePeriodic() {
        for (var module : modules) {
            module.periodic();
        }
    }

    public void updateModuleSamples() { // allows all signals to get sampled together
        sampleTimestamps = modules[0].getOdometryTimestamps();
        sampleCount = sampleTimestamps.length;
        
        sampleModulePositions = new SwerveModulePosition[sampleCount][4];
        sampleModuleDeltas = new SwerveModulePosition[sampleCount][4];
        
        for (int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
                );
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }
            sampleModulePositions[i] = modulePositions;
            sampleModuleDeltas[i] = moduleDeltas;
        }
    }

    public int getSampleCount() {
        return sampleCount;
    }

    public double[] getSampleTimestamps() {
        return sampleTimestamps;
    }

    public SwerveModulePosition[][] getSampleModulePositions() {
        return sampleModulePositions;
    }

    public SwerveModulePosition[][] getSampleModuleDeltas() {
        return sampleModuleDeltas;
    }
    
    // ————— utils ————— //
    
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }
}
