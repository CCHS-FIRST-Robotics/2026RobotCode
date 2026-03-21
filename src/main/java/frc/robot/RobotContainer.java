package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import java.io.*;
import choreo.auto.AutoChooser;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.subsystems.climber.*;
import frc.robot.utils.*;
import frc.robot.utils.ShootUtil.ShooterState;

// LED IMPORTS
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

@SuppressWarnings("unused")
public class RobotContainer {
    // ————— controllers ————— //
    private final Controller controller = new Controller(0);

    // ————— subsystems ————— //
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    // ————— LED OBJECTS ————— //
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
    private final int LED_COUNT = 60; // CHANGE THIS to your actual IP30 strip length
    private final int LED_PWM_PORT = 0; // The PWM port on the RoboRIO

    // ————— utils ————— //
    private CommandFactory commandFactory;
    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    private FuelSim fuelSimulation;

    // ————— testing variables ————— //
    private double shooterVelocity = 0;
    private double hoodAngle = 0;

    @AutoLogOutput
    private double hopperVelocity = 0;
    @AutoLogOutput
    private double kickerVelocity = 0;

    public RobotContainer() {
        // ————— Initialize LEDs ————— //
        m_led = new AddressableLED(LED_PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        switch (Constants.CURRENT_MODE) {
            case REAL: 
                drive = new Drive(
                    new ModuleIOTalonFXReal(DriveConstants.SWERVE_MODULE_CONSTANTS[0]),
                    new ModuleIOTalonFXReal(DriveConstants.SWERVE_MODULE_CONSTANTS[1]),
                    new ModuleIOTalonFXReal(DriveConstants.SWERVE_MODULE_CONSTANTS[2]),
                    new ModuleIOTalonFXReal(DriveConstants.SWERVE_MODULE_CONSTANTS[3])
                );
                poseEstimator = new PoseEstimator(
                    new GyroIOPigeon2(),
                    new CameraIOPhotonVision[] {
                        new CameraIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0),
                        new CameraIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
                    }, 
                    drive, 
                    Constants.ROBOT_START_POSE
                );
                intake = new Intake(
                    new IntakeIOReal(FuelConstants.INTAKE_MOTOR_ID), 
                    new PivotIOReal(FuelConstants.PIVOT_MOTOR_ID)
                );
                hopper = new Hopper(new HopperIOReal(FuelConstants.HOPPER_MOTOR_ID));
                shooter = new Shooter(
                    new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID), 
                    new HoodIOReal(FuelConstants.HOOD_MOTOR_ID), 
                    new KickerIOReal(FuelConstants.KICKER_MOTOR_ID)
                );
                break;
            case SIM: 
                configureSimulation();
                drive = new Drive(
                    new ModuleIOTalonFXSim(DriveConstants.SWERVE_MODULE_CONSTANTS[0], driveSimulation.getModules()[0]),
                    new ModuleIOTalonFXSim(DriveConstants.SWERVE_MODULE_CONSTANTS[1], driveSimulation.getModules()[1]),
                    new ModuleIOTalonFXSim(DriveConstants.SWERVE_MODULE_CONSTANTS[2], driveSimulation.getModules()[2]),
                    new ModuleIOTalonFXSim(DriveConstants.SWERVE_MODULE_CONSTANTS[3], driveSimulation.getModules()[3])
                );
                poseEstimator = new PoseEstimator(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new CameraIOPhotonVision[] {
                        new CameraIOPhotonVisionSim(VisionConstants.camera0Name, VisionConstants.robotToCamera0, driveSimulation::getSimulatedDriveTrainPose),
                        new CameraIOPhotonVisionSim(VisionConstants.camera1Name, VisionConstants.robotToCamera1, driveSimulation::getSimulatedDriveTrainPose)
                    },
                    drive, 
                    Constants.ROBOT_START_POSE
                );
                intake = new Intake(new IntakeIOSim(), new PivotIOSim());
                hopper = new Hopper(new HopperIOSim());
                shooter = new Shooter(new ShooterIOSim(), new HoodIOSim(), new KickerIOSim());
                break;
            default: 
                drive = new Drive(new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                poseEstimator = new PoseEstimator(new GyroIO() {}, new CameraIO[] {new CameraIO() {}, new CameraIO() {}}, drive, new Pose2d());
                intake = new Intake(new IntakeIO() {}, new PivotIO() {});
                hopper = new Hopper(new HopperIO() {});
                shooter = new Shooter(new ShooterIO() {}, new HoodIO() {}, new KickerIO() {});
                break;
        }

        drive.setPoseEstimator(poseEstimator);

        commandFactory = new CommandFactory(
            controller, 
            drive, 
            poseEstimator, 
            intake, 
            hopper, 
            shooter,
            fuelSimulation
        );

        configureButtonBindings();
        configureAutos();
    }

    /**
     * Logic to update LEDs. 
     * Call this from Robot.java -> robotPeriodic()
     */
    public void updateLEDs() {
        // Rainbow effect logic
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set HSV (Hue, Saturation, Value)
            // Value is set to 128 (50% brightness) to stay under the 2A VRM limit.
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        
        m_rainbowFirstPixelHue = (m_rainbowFirstPixelHue + 3) % 180;
        m_led.setData(m_ledBuffer);
    }

    private void configureButtonBindings() {
        drive.setDefaultCommand(commandFactory.getDriveWithJoysticksCommand());
    }

    //     if (Constants.COMPETITION) {
    //         controller.x().whileTrue(Commands.run(() -> drive.xLock()));

    //         controller.rightStick().whileTrue(
    //             commandFactory.getSlowDriveCommand(
    //                 MetersPerSecond.of(1), 
    //                 RadiansPerSecond.of(1 / DriveConstants.TRACK_RADIUS), 
    //                 DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
    //                 DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
    //             )
    //         );

    //         controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(
    //             commandFactory.getDriveAndIntakeCommand()
    //         );

    //         controller.leftTrigger().and(controller.rightTrigger()).whileTrue(
    //             commandFactory.getDriveAndIntakeAndShootCommand()
    //         );

    //         controller.leftTrigger().negate().and(controller.rightTrigger()).whileTrue(
    //             commandFactory.getDriveAndShootCommand(true)
    //         );

    //         controller.rightBumper().whileTrue(commandFactory.getIntakeCommand());
    //         controller.y().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_UP_ANGLE));
    //         controller.a().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE));
    //         controller.b().onTrue(new InstantCommand(() -> Constants.TRENCH_ALIGN = !Constants.TRENCH_ALIGN));
    //     }
    // }

    private void configureAutos() {
        autoGenerator = new AutoGenerator(drive, poseEstimator, intake, hopper, shooter, driveSimulation, commandFactory);
        autoChooser = new AutoChooser();
        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("BackUpAndShoot", () -> autoGenerator.backUpAndShoot());
        autoChooser.select("BackUpAndShoot");
        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void autonomousPeriodic() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL || Constants.REALISTIC_SIM) {
            Logger.recordOutput("outputs/fieldInfo/remainingShiftTime", HubUtil.timeRemainingInCurrentShift().orElse(Seconds.of(-1)));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public void teleopPeriodic() {
        // Standard Teleop Logging
    }

    private void configureSimulation() {
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, Constants.ROBOT_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        fuelSimulation = new FuelSim();
        fuelSimulation.registerRobot(
            DriveConstants.WIDTH_X.in(Meters),
            DriveConstants.WIDTH_Y.in(Meters),
            Inches.of(6).in(Meters),
            () -> poseEstimator.getPose(),
            () -> drive.getFieldRelativeSpeeds()
        );
        fuelSimulation.start();
    }

    public void simulationPeriodic() {
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) return;
        SimulatedArena.getInstance().simulationPeriodic();
        fuelSimulation.stepSim();
    }

    public void resetSimulation() {
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) return;
        Pose2d startPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
            Constants.ROBOT_START_POSE : 
            Constants.FieldConstants.calculateAllianceFlippedPose(Constants.ROBOT_START_POSE);
        driveSimulation.setSimulationWorldPose(startPose);
        poseEstimator.resetPosition(startPose);
        fuelSimulation.clearFuel();
    }
}