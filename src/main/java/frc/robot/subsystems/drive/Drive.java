/**
 * Original code
 */

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import choreo.trajectory.SwerveSample;
import org.littletonrobotics.junction.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.Constants;

public class Drive extends SubsystemBase {    
    private enum DRIVE_MODE {
        DISABLED,
        CHARACTERIZING,
        POSITION,
        VELOCITY
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

    private final PIDController xPIDPosition = new PIDController(2.5, 0, 0);
    private final PIDController yPIDPosition = new PIDController(2.5, 0, 0);
    private final PIDController thetaPIDPosition = new PIDController(4, 0, 0);

    private final PIDController xPIDChoreo = new PIDController(5, 0, 0);
    private final PIDController yPIDChoreo = new PIDController(5, 0, 0);
    private final PIDController thetaPIDChoreo = new PIDController(5, 0, 0);

    private Pose2d positionSetpoint = new Pose2d();
    private ChassisSpeeds velocitySetpoint = new ChassisSpeeds();

    boolean usingChoreo = false;

    // ————— velocity ————— //

    private ChassisSpeeds speeds = new ChassisSpeeds();
    private ChassisSpeeds prevSpeeds = new ChassisSpeeds();
    private ChassisSpeeds speedsOutput = new ChassisSpeeds();

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
        
        // allow position PID to turn in the correct direction
        thetaPIDPosition.enableContinuousInput(-Math.PI, Math.PI);
        thetaPIDChoreo.enableContinuousInput(-Math.PI, Math.PI);

        thetaPIDPosition.setTolerance(0.15);

        // allow PID to be tuned through elastic
        SmartDashboard.putData("smartDashboard/PID/drivePIDPosition/x", xPIDPosition);
        SmartDashboard.putData("smartDashboard/PID/drivePIDPosition/y", yPIDPosition);
        SmartDashboard.putData("smartDashboard/PID/drivePIDPosition/theta", thetaPIDPosition);
        SmartDashboard.putData("smartDashboard/PID/drivePIDChoreo/x", xPIDChoreo);
        SmartDashboard.putData("smartDashboard/PID/drivePIDChoreo/y", yPIDChoreo);
        SmartDashboard.putData("smartDashboard/PID/drivePIDChoreo/theta", thetaPIDChoreo);
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            controlMode = DRIVE_MODE.DISABLED;
        }

        // log the real module states and speeds
        SwerveModuleState[] moduleStatesOutput = getModuleStates();
        Logger.recordOutput("outputs/drive/moduleStatesOutput", moduleStatesOutput);
        speedsOutput = DriveConstants.KINEMATICS.toChassisSpeeds(moduleStatesOutput);
        Logger.recordOutput("outputs/drive/speedsOutput", speedsOutput);

        // run the selected control mode
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
                Logger.recordOutput("outputs/drive/targetPose", positionSetpoint);

                double xOutput;
                double yOutput;
                double thetaOutput;

                // get PID outputs
                if (!usingChoreo) {
                    xOutput = xPIDPosition.calculate(poseEstimator.getPose().getX(), positionSetpoint.getX());
                    yOutput = yPIDPosition.calculate(poseEstimator.getPose().getY(), positionSetpoint.getY());
                    thetaOutput = thetaPIDPosition.calculate(poseEstimator.getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());
                } else {
                    xOutput = xPIDChoreo.calculate(poseEstimator.getPose().getX(), positionSetpoint.getX());
                    yOutput = yPIDChoreo.calculate(poseEstimator.getPose().getY(), positionSetpoint.getY());
                    thetaOutput = thetaPIDChoreo.calculate(poseEstimator.getPose().getRotation().getRadians(), positionSetpoint.getRotation().getRadians());
                }

                // create robot relative chassisspeeds object
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xOutput + velocitySetpoint.vxMetersPerSecond,
                    yOutput + velocitySetpoint.vyMetersPerSecond,
                    thetaOutput + velocitySetpoint.omegaRadiansPerSecond,
                    poseEstimator.getPose().getRotation()
                );
                // fallthrough to VELOCITY case; no break statement needed
            case VELOCITY: 
                if (!usingChoreo) { // don't limit velocity or acceleration if using choreo (limits should be configured in the auto path)
                    speeds = new ChassisSpeeds( // clamp velocities
                        MathUtil.clamp(speeds.vxMetersPerSecond, -DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond), DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond)), 
                        MathUtil.clamp(speeds.vyMetersPerSecond, -DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond), DriveConstants.ALLOWED_LINEAR_SPEED.in(MetersPerSecond)), 
                        MathUtil.clamp(speeds.omegaRadiansPerSecond, -DriveConstants.ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond), DriveConstants.ALLOWED_ANGULAR_SPEED.in(RadiansPerSecond))
                    );

                    speeds = new ChassisSpeeds( // clamp accelerations
                        clampAcceleration(
                            speeds.vxMetersPerSecond, 
                            prevSpeeds.vxMetersPerSecond, 
                            DriveConstants.ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * Constants.PERIOD
                        ),
                        clampAcceleration(
                            speeds.vyMetersPerSecond, 
                            prevSpeeds.vyMetersPerSecond, 
                            DriveConstants.ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) * Constants.PERIOD
                        ),
                        clampAcceleration(
                            speeds.omegaRadiansPerSecond, 
                            prevSpeeds.omegaRadiansPerSecond, 
                            DriveConstants.ALLOWED_ANGULAR_ACCEL.in(RadiansPerSecondPerSecond) * Constants.PERIOD
                        )
                    );
                }
            
                speeds = ChassisSpeeds.discretize(speeds, Constants.PERIOD); // explaination: https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/30
                
                Logger.recordOutput("outputs/drive/speedsInput", speeds);
                prevSpeeds = speeds;

                SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds); // convert speeds to module states
                
                if (!usingChoreo) {
                    SwerveDriveKinematics.desaturateWheelSpeeds( // renormalize wheel speeds
                        moduleStates, 
                        speeds,
                        DriveConstants.ALLOWED_LINEAR_SPEED, 
                        DriveConstants.ALLOWED_LINEAR_SPEED, 
                        DriveConstants.ALLOWED_ANGULAR_SPEED
                    );
                } else {
                    SwerveDriveKinematics.desaturateWheelSpeeds( // renormalize wheel speeds
                        moduleStates, 
                        speeds,
                        DriveConstants.MAX_THEORETICAL_LINEAR_SPEED, 
                        DriveConstants.MAX_THEORETICAL_LINEAR_SPEED, 
                        DriveConstants.MAX_THEORETICAL_ANGULAR_SPEED
                    );
                }

                // run modules
                for (int i = 0; i < 4; i++) {
                    modules[i].runSetpoint(moduleStates[i]);
                }
                Logger.recordOutput("outputs/drive/moduleStatesInput", moduleStates);
                break;
        }
    }

    // ————— run control mode ————— //

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
        usingChoreo = false;
        positionSetpoint = pose;
        velocitySetpoint = new ChassisSpeeds();
    }

    public void runPositionChoreo(SwerveSample sample) {
        controlMode = DRIVE_MODE.POSITION;
        usingChoreo = true;
        positionSetpoint = sample.getPose();
        velocitySetpoint = sample.getChassisSpeeds();
    }

    public void runVelocity(ChassisSpeeds speedsInput) {
        controlMode = DRIVE_MODE.VELOCITY;
        usingChoreo = false;
        speeds = speedsInput;
    }

    // ————— general public functions ————— //

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return speedsOutput;
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(speedsOutput, poseEstimator.getPose().getRotation());
    }

    public PIDController getXPositionController() {
        return xPIDPosition;
    }

    public PIDController getYPositionController() {
        return yPIDPosition;
    }

    public PIDController getThetaPositionController() {
        return thetaPIDPosition;
    }

    @AutoLogOutput(key = "outputs/drive/atThetaSetpoint")
    public boolean atThetaSetpoint() {
        return thetaPIDPosition.atSetpoint();
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
        for (Module module : modules) {
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

    public void xLock() {
        runCharacterization(
            new Voltage[] {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)}, 
            new Angle[] {Rotations.of(0.125), Rotations.of(0.325), Rotations.of(0.325), Rotations.of(0.125)}
        );
    }

    public double clampAcceleration(double velocity, double prevVelocity, double maxAcceleration) {
        return MathUtil.clamp(velocity, prevVelocity - maxAcceleration, prevVelocity + maxAcceleration);
    }
}