package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.math.geometry.*;
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
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;

public class RobotContainer {
    // ————— controllers ————— //

    private final Controller controller = new Controller(0);

    // ————— subsystems ————— //

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Shooter shooter;

    // ————— utils ————— //

    private CommandFactory commandFactory;
    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    private FuelSim fuelSimulation;

    // ————— testing variables ————— //

    @AutoLogOutput
    double shooterVelocity = 0;
    @AutoLogOutput
    double kickerVelocity = 0;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL: // real robot, instantiate hardware IO implementations
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
                shooter = new Shooter(
                    new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID, FuelConstants.SHOOTER_FOLLOWER_ID), 
                    new KickerIOReal(FuelConstants.KICKER_MOTOR_ID)
                );
                break;
            case SIM: // sim robot, instantiate physics sim IO implementations
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
                        new CameraIOPhotonVisionSim(
                            VisionConstants.camera0Name, 
                            VisionConstants.robotToCamera0, 
                            driveSimulation::getSimulatedDriveTrainPose // this is why vision and combined estimators also have collision
                        ),
                        new CameraIOPhotonVisionSim(
                            VisionConstants.camera1Name, 
                            VisionConstants.robotToCamera1, 
                            driveSimulation::getSimulatedDriveTrainPose
                        )
                    },
                    drive, 
                    Constants.ROBOT_START_POSE
                );
                intake = new Intake(
                    new IntakeIOSim(), 
                    new PivotIOSim()
                );
                shooter = new Shooter(
                    new ShooterIOSim(),
                    new KickerIOSim()
                );
                break;
            default: // replayed robot, disable IO implementations
                drive = new Drive(
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {}
                );
                poseEstimator = new PoseEstimator(
                    new GyroIO() {}, 
                    new CameraIO[] {
                        new CameraIO() {}, 
                        new CameraIO() {}
                    }, 
                    drive, 
                    new Pose2d()
                );
                intake = new Intake(
                    new IntakeIO() {}, 
                    new PivotIO() {}
                );
                shooter = new Shooter(
                    new ShooterIO() {}, 
                    new KickerIO() {}
                );
                break;
        }

        drive.setPoseEstimator(poseEstimator);

        commandFactory = new CommandFactory(
            controller, 
            drive, 
            poseEstimator, 
            intake, 
            shooter,
            fuelSimulation
        );

        configureButtonBindings();
        configureAutos();
    }

    private void configureButtonBindings() {
        // drive
        drive.setDefaultCommand(commandFactory.getDriveWithJoysticksCommand());

        switch (Constants.CURRENT_BUTTON_BINDINGS) {
            case COMPETITION: 
                // x-lock
                controller.x().whileTrue(
                    Commands.run(() -> drive.xLock())
                );

                // drive slow // ! wasn't used
                controller.rightStick().whileTrue( // remapped as gamesir R4
                    commandFactory.getSlowDriveCommand(
                        MetersPerSecond.of(1), 
                        RadiansPerSecond.of(1 / DriveConstants.TRACK_RADIUS), 
                        DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
                        DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
                    )
                );

                // drive and intake
                controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(
                    commandFactory.getDriveAndIntakeCommand()
                );

                // drive and intake and shoot
                controller.leftTrigger().and(controller.rightTrigger()).whileTrue(
                    commandFactory.getDriveAndIntakeAndShootCommand()
                );

                // drive and shoot
                controller.leftTrigger().negate().and(controller.rightTrigger()).whileTrue(
                    commandFactory.getDriveAndShootCommand(true)
                );

                controller.rightBumper().whileTrue(commandFactory.getIntakeCommand());

                // pivot // ! make it a toggle leftbumper
                controller.y().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_UP_ANGLE));
                controller.a().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE));

                controller.b().onTrue(new InstantCommand(() -> Constants.TRENCH_ALIGN = !Constants.TRENCH_ALIGN));
                break;
            case TESTING_BPS: 
                controller.rightTrigger().whileTrue(
                    new StartEndCommand(
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                            shooter.setKickerVelocity(RotationsPerSecond.of(kickerVelocity));
                        }, 
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(0));
                            shooter.setKickerVelocity(RotationsPerSecond.of(0));
                        }
                    )
                );

                // increment shooter and kicker velocity
                controller.x().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity += 5;
                    })
                );
                controller.b().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity -= 5;
                    })
                );
                controller.y().onTrue(
                    new InstantCommand(() -> {
                        kickerVelocity += 5;
                    })
                );
                controller.a().onTrue(
                    new InstantCommand(() -> {
                        kickerVelocity -= 5;
                    })
                );
                break;
            case TESTING_SHOOTER_MAP:
                controller.x().whileTrue( // orient the robot
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getDriveWithJoysticksShooterCommand())
                );

                controller.y().whileTrue( // shoot
                    commandFactory.getShootCommand(
                        () -> RotationsPerSecond.of(shooter.shooterIOInputs.velocitySetpoint), 
                        () -> true
                    )
                );

                controller.a().onTrue( // print everything
                    new InstantCommand(() -> 
                        {
                            System.out.println("SHOOTER_VELOCITY_MAP.put(DISTANCE, " + shooter.shooterIOInputs.velocity + ");");
                        }
                    )
                );

                // increment the shooter velocity
                controller.leftTrigger().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity -= 1.25;
                        shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                    })
                );
                controller.rightTrigger().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity += 1.25;
                        shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                    })
                );
                break;
        }

        // ————— simulation bindings ————— //

        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) {
            controller.b().onTrue(new InstantCommand(() -> fuelSimulation.clearFuel())); // ! make this be a button on elastic
        }
    }

    // ————— autonomous ————— //

    private void configureAutos() {
        autoGenerator = new AutoGenerator(
            drive, 
            poseEstimator, 
            intake, 
            shooter, 
            driveSimulation, 
            commandFactory
        );
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("BackUpAndShoot", () -> autoGenerator.backUpAndShoot());
        autoChooser.addRoutine("BeMeanBottom", () -> autoGenerator.beMeanBottom());
        autoChooser.addRoutine("BeMeanTop", () -> autoGenerator.beMeanTop());
        autoChooser.addRoutine("CenterFuelBottom", () -> autoGenerator.centerFuelBottom());
        autoChooser.addRoutine("CenterFuelTop", () -> autoGenerator.centerFuelTop());
        autoChooser.addRoutine("OutpostFuel", () -> autoGenerator.outpostFuel());

        autoChooser.select("BackUpAndShoot"); // picks a default auto

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void autonomousPeriodic() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL || Constants.REALISTIC_SIM) {
            Logger.recordOutput("outputs/fieldInfo/remainingShiftTime", HubUtil.timeRemainingInCurrentShift().orElse(Seconds.of(-1)));
            Logger.recordOutput("outputs/fieldInfo/currentShift", HubUtil.getCurrentShift().orElse(HubUtil.Shift.NO_SHIFT));
            Logger.recordOutput("outputs/fieldInfo/hubActive", HubUtil.isActive());
            
            Logger.recordOutput(
            "outputs/simulation/fuelSimulation/hubScore", 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
                FuelSim.BLUE_HUB.getScore() : 
                FuelSim.RED_HUB.getScore()
            );
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    // ————— teleop ————— //

    public void teleopPeriodic() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL || Constants.REALISTIC_SIM) {
            Logger.recordOutput("outputs/fieldInfo/autoWinner", HubUtil.getAutoWinner());
            Logger.recordOutput("outputs/fieldInfo/remainingShiftTime", Math.round(HubUtil.timeRemainingInCurrentShift().orElse(Seconds.of(-1)).in(Seconds)));
            Logger.recordOutput("outputs/fieldInfo/currentShift", HubUtil.getCurrentShift().orElse(HubUtil.Shift.NO_SHIFT));
            Logger.recordOutput("outputs/fieldInfo/hubActive", HubUtil.isActive());
            
            Logger.recordOutput(
            "outputs/simulation/fuelSimulation/hubScore", 
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
                FuelSim.BLUE_HUB.getScore() : 
                FuelSim.RED_HUB.getScore()
            );
        }
    }

    // ————— simulation ————— //

    private void configureSimulation() {
        // drive
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, Constants.ROBOT_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        Constants.FieldConstants.Zones.logAllZones();

        // drive
        fuelSimulation = new FuelSim();
        fuelSimulation.registerRobot(
            DriveConstants.WIDTH_X.in(Meters),
            DriveConstants.WIDTH_Y.in(Meters),
            Inches.of(6).in(Meters),
            () -> poseEstimator.getPose(),
            () -> drive.getFieldRelativeSpeeds()
        );
        fuelSimulation.registerIntake(
            -DriveConstants.WIDTH_X.div(2).in(Meters) - FuelConstants.INTAKE_WIDTH_X.in(Meters),
            -DriveConstants.WIDTH_X.div(2).in(Meters),
            -DriveConstants.WIDTH_Y.div(2).in(Meters),
            DriveConstants.WIDTH_Y.div(2).in(Meters),
            () -> {
                return intake.getIntakeOn() && (Constants.REALISTIC_SIM ? !intake.getHopperFull() : true);
            }, 
            () -> {
                if (Constants.REALISTIC_SIM) {
                    if (intake.getHopperFull()) {
                        return;
                    }
                    intake.addHopperFuel();
                }
            }
        );
        fuelSimulation.setSubticks(1);
        fuelSimulation.start();

        if (Constants.REALISTIC_SIM) {
            fuelSimulation.spawnStartingFuel();
        }
    }

    public void simulationPeriodic() {
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) { // not sure if this has to be here if it's only called in simulationPeriodic
            return;
        }

        // drive
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("outputs/simulation/fieldSimulation/robotPosition", driveSimulation.getSimulatedDriveTrainPose());

        // fuel
        fuelSimulation.stepSim();
    }

    public void resetSimulation() {
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) {
            return;
        }

        // drive
        Pose2d startPose = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 
        Constants.ROBOT_START_POSE : 
        Constants.FieldConstants.calculateAllianceFlippedPose(Constants.ROBOT_START_POSE);

        driveSimulation.setSimulationWorldPose(startPose);
        poseEstimator.resetPosition(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();

        // fuel
        fuelSimulation.clearFuel();

        if (Constants.REALISTIC_SIM) {
            fuelSimulation.spawnStartingFuel();
        }
    }
}