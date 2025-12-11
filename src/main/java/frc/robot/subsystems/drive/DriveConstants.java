package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public class DriveConstants {
    public static final CANBus CAN_BUS = new CANBus("", "./logs/example.hoot");
    public static final double ODOMETRY_FREQUENCY = CAN_BUS.isNetworkFD() ? 250.0 : 100.0;

    // ————— motors ————— //

    private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement TURN_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

    private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration();
    private static final TalonFXConfiguration TURN_CONFIG = new TalonFXConfiguration()
    .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(60))
        .withStatorCurrentLimitEnable(true)
    );

    private static final int[] DRIVE_MOTOR_IDS = {11, 21, 31, 41};
    private static final int[] TURN_MOTOR_IDS = {12, 22, 32, 42};
    private static final boolean[] MOTOR_INVERSIONS = {true, true, true, true};
    private static final boolean[] SIDE_INVERSIONS = {false, false, true, true};
    
    // ————— encoders ————— //
    
    private static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration();
    
    private static final int[] ENCODER_IDS = {1, 2, 3, 4};
    private static final boolean[] ENCODER_INVERSIONS = {false, false, false, false};
    private static final Angle[] ENCODER_OFFSETS = {
        Rotations.of(0.03271484375),
        Rotations.of(-0.38720703125),
        Rotations.of(-0.483154296875),
        Rotations.of(-0.283203125)
    };

    // ————— gyro ————— //
    
    private static final Pigeon2Configuration PIGEON_CONFIG = null;
    
    private static final int PIGEON_ID = 5;

    // ————— PID ————— //

    private static final Slot0Configs DRIVE_PID = new Slot0Configs()
    .withKP(0.000062333)
    .withKI(0)
    .withKD(0)
    .withKS(0.03422)
    .withKV(0.13259)
    .withKA(0.025003);
    
    private static final Slot0Configs TURN_PID = new Slot0Configs()
    .withKP(15)
    .withKI(0)
    .withKD(0)
    .withKS(0)
    .withKV(0)
    .withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final ClosedLoopOutputType DRIVE_PID_TYPE = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType TURN_PID_TYPE = ClosedLoopOutputType.Voltage; // the other option is torque current FOC
    private static final SteerFeedbackType TURN_PID_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder; // when not pro-licensed, fused/sync automatically fall back to remote

    // ————— modules ————— //

    private static final Distance WHEEL_RADIUS = Inches.of(2);
    public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;
    private static final double MODULE_COUPLE_RATIO = 3.125; // ! check this
    private static final double MODULE_DRIVE_GEAR_RATIO = 5.902777777777778;
    private static final double MODULE_TURN_GEAR_RATIO = 18.75;
    
    // ————— drivetrain ————— //

    // ! these are probably wrong
    public static final Mass ROBOT_WEIGHT = Pounds.of(74.088);
    public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);
    public static final Distance TRACK_WIDTH_X = Inches.of(26.5);
    public static final Distance TRACK_WIDTH_Y = Inches.of(26.5); 
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] { // using the chassisspeeds coordinate plane
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // FL
        new Translation2d(TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0), // FR
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, TRACK_WIDTH_Y.in(Meters) / 2.0), // BL
        new Translation2d(-TRACK_WIDTH_X.in(Meters) / 2.0, -TRACK_WIDTH_Y.in(Meters) / 2.0) // BR
    };
    public static final double TRACK_RADIUS = Math.max(
        Math.max(
            Math.hypot(MODULE_TRANSLATIONS[0].getX(), MODULE_TRANSLATIONS[0].getY()),
            Math.hypot(MODULE_TRANSLATIONS[1].getX(), MODULE_TRANSLATIONS[1].getY())
        ),
        Math.max(
            Math.hypot(MODULE_TRANSLATIONS[2].getX(), MODULE_TRANSLATIONS[2].getY()),
            Math.hypot(MODULE_TRANSLATIONS[3].getX(), MODULE_TRANSLATIONS[3].getY())
        )
    );
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    
    public static final LinearVelocity MAX_THEORETICAL_LINEAR_SPEED = MetersPerSecond.of(5.41); // ! erm, no
    public static final LinearVelocity MAX_ALLOWED_LINEAR_SPEED = MetersPerSecond.of(5);
    public static final AngularVelocity MAX_ALLOWED_ANGULAR_SPEED = RadiansPerSecond.of(MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond) / DriveConstants.TRACK_RADIUS);
    private static final Current SLIP_CURRENT = Amps.of(120.0);

    // these are only used for simulation // ! https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template/
    private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia TURN_INERTIA = KilogramSquareMeters.of(0.01);
    private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);
    private static final Voltage TURN_FRICTION_VOLTAGE = Volts.of(0.2); // simulated voltage necessary to overcome friction

    // ————— compilation ————— //

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
    .withCANBusName(CAN_BUS.getName())
    .withPigeon2Id(PIGEON_ID)
    .withPigeon2Configs(PIGEON_CONFIG);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> 
    DRIVE_CONSTANT_CREATOR = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
    .withDriveMotorGearRatio(MODULE_DRIVE_GEAR_RATIO)
    .withSteerMotorGearRatio(MODULE_TURN_GEAR_RATIO)
    .withCouplingGearRatio(MODULE_COUPLE_RATIO)
    .withWheelRadius(WHEEL_RADIUS)
    .withSteerMotorGains(TURN_PID)
    .withDriveMotorGains(DRIVE_PID)
    .withSteerMotorClosedLoopOutput(TURN_PID_TYPE)
    .withDriveMotorClosedLoopOutput(DRIVE_PID_TYPE)
    .withSlipCurrent(SLIP_CURRENT)
    .withSpeedAt12Volts(MAX_ALLOWED_LINEAR_SPEED)
    .withDriveMotorType(DRIVE_MOTOR_TYPE)
    .withSteerMotorType(TURN_MOTOR_TYPE)
    .withFeedbackSource(TURN_PID_FEEDBACK_TYPE)
    .withDriveMotorInitialConfigs(DRIVE_CONFIG)
    .withSteerMotorInitialConfigs(TURN_CONFIG)
    .withEncoderInitialConfigs(ENCODER_CONFIG)
    .withSteerInertia(TURN_INERTIA)
    .withDriveInertia(DRIVE_INERTIA)
    .withSteerFrictionVoltage(TURN_FRICTION_VOLTAGE)
    .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

    @SuppressWarnings("unchecked")
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[]
    SWERVE_MODULE_CONSTANTS = new SwerveModuleConstants[4];

    static{
        for(int i = 0; i < 4; i++){
            SWERVE_MODULE_CONSTANTS[i] = DRIVE_CONSTANT_CREATOR.createModuleConstants(
                TURN_MOTOR_IDS[i],
                DRIVE_MOTOR_IDS[i],
                ENCODER_IDS[i],
                ENCODER_OFFSETS[i],
                Meters.of(MODULE_TRANSLATIONS[i].getX()), 
                Meters.of(MODULE_TRANSLATIONS[i].getY()),
                SIDE_INVERSIONS[i],
                MOTOR_INVERSIONS[i],
                ENCODER_INVERSIONS[i]
            );
        }
    }
                    
    public static final DriveTrainSimulationConfig DRIVE_SIMULATION_CONFIG = DriveTrainSimulationConfig.Default()
    .withRobotMass(DriveConstants.ROBOT_WEIGHT)
    .withCustomModuleTranslations(MODULE_TRANSLATIONS)
    .withGyro(COTS.ofPigeon2())
    .withSwerveModule(new SwerveModuleSimulationConfig(
        DCMotor.getKrakenX60(1),
        DCMotor.getFalcon500(1),
        SWERVE_MODULE_CONSTANTS[0].DriveMotorGearRatio,
        SWERVE_MODULE_CONSTANTS[0].SteerMotorGearRatio,
        Volts.of(SWERVE_MODULE_CONSTANTS[0].DriveFrictionVoltage),
        Volts.of(SWERVE_MODULE_CONSTANTS[0].SteerFrictionVoltage),
        Meters.of(SWERVE_MODULE_CONSTANTS[0].WheelRadius),
        KilogramSquareMeters.of(SWERVE_MODULE_CONSTANTS[0].SteerInertia),
        DriveConstants.WHEEL_COEFFICIENT_OF_FRICTION)
    );
}