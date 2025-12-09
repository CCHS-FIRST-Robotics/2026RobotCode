package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.*;

import choreo.trajectory.SwerveSample;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {    
    private enum DRIVE_MODE {
        DISABLED,
        POSITION,
        VELOCITY,
        CHARACTERIZING
    };
    private DRIVE_MODE controlMode = DRIVE_MODE.DISABLED;
    
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    
    // ————— odometry ————— //

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    static final Lock odometryLock = new ReentrantLock();
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, rawGyroRotation, lastModulePositions, new Pose2d());
    
    // ————— characterization ————— //

    private Voltage[] characterizationVolts = {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)};
    private Angle[] characterizationPositions = {Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)};
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(1), // ramp rate
            Volts.of(1.75), // step voltage
            Seconds.of(4), // timeout
            (state) -> Logger.recordOutput("drive/sysIdState", state.toString()) // send the data to advantagekit
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

    // ————— position ————— //

    private Pose2d positionSetpoint = new Pose2d();
    private Twist2d twistSetpoint = new Twist2d();
    private final PIDController xPID = new PIDController(5, 0, 0);
    private final PIDController yPID = new PIDController(5, 0, 0);
    private final PIDController oPID = new PIDController(10, 0, 0);

    // ————— velocity ————— //

    private ChassisSpeeds speeds = new ChassisSpeeds();

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO
    ) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, DriveConstants.SWERVE_MODULE_CONSTANTS[0]);
        modules[1] = new Module(frModuleIO, 1, DriveConstants.SWERVE_MODULE_CONSTANTS[1]);
        modules[2] = new Module(blModuleIO, 2, DriveConstants.SWERVE_MODULE_CONSTANTS[2]);
        modules[3] = new Module(brModuleIO, 3, DriveConstants.SWERVE_MODULE_CONSTANTS[3]);

        // Usage reporting for swerve template // ! tf is this
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        PhoenixOdometryThread.getInstance().start(); // start odometry thread
        
        oPID.enableContinuousInput(-Math.PI, Math.PI); // allows position PID to turn in the correct direction
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = DRIVE_MODE.DISABLED;
        }

        // ————— odometry ————— //

        odometryLock.lock(); // prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("drive/gyro", gyroInputs);
        for (var module : modules) { // ! uhhhhhhhhhh
            module.periodic();
        }
        odometryLock.unlock();

        // Update odometry  // ! hmmmmmm
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = DriveConstants.KINEMATICS.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

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
                double xOutput = xPID.calculate(getPose().getX(), positionSetpoint.getX());
                double yOutput = yPID.calculate(getPose().getY(), positionSetpoint.getY());
                double oOutput = oPID.calculate(getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());

                // create chassisspeeds object with FOC
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    twistSetpoint.dx + xOutput,
                    twistSetpoint.dy + yOutput,
                    twistSetpoint.dtheta + oOutput,
                    getPose().getRotation()
                );
                // fallthrough to VELOCITY case; no break statement needed
            case VELOCITY: 
                speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD); // explaination: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
                
                SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds); // convert speeds to module states
                SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_THEORETICAL_LINEAR_SPEED); // renormalize wheel speeds

                // run modules
                for (int i = 0; i < 4; i++) {
                    modules[i].runSetpoint(moduleStates[i]);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", moduleStates);
                break;
        }
    }

    // ————— functions for running modules ————— //

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

    // ————— functions for sysid ————— // 

    public Command sysIdFull() {
        return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)
            .andThen(sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward))
            .andThen(sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    // ! ————— functions for odometry ————— //

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured") // ! don't use this annotation
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured") // ! don't use this annotation
    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}
