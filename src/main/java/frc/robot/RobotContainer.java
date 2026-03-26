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
import frc.robot.subsystems.leds.*;
import frc.robot.utils.*;

@SuppressWarnings("unused")
public class RobotContainer {
    // ————— controllers ————— //

    private final Controller controller = new Controller(0);

    // ————— subsystems ————— //

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Shooter shooter;

    private final LedStrip ledStrip;

    // ————— utils ————— //

    private CommandFactory commandFactory;
    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    private FuelSim fuelSimulation;

    // ————— testing variables ————— //

    @AutoLogOutput(key = "outputs/fuelIO/shooter/shooterVelocity")
    private double shooterVelocity = 0;
    @AutoLogOutput(key = "outputs/fuelIO/shooter/kickerVelocity")
    private double kickerVelocity = 0;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL: // real robot, instantiate hardware IO implementations
                if (Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
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
                            new CameraIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1),
                            new CameraIOPhotonVision(VisionConstants.camera2Name, VisionConstants.robotToCamera2)
                        }, 
                        drive, 
                        Constants.ROBOT_START_POSE
                    );
                } else {
                    drive = null;
                    poseEstimator = null;
                }

                if (Constants.INSTANTIATE_INTAKE) {
                    intake = new Intake(
                        new IntakeIOReal(FuelConstants.INTAKE_MOTOR_ID), 
                        new PivotIOReal(FuelConstants.PIVOT_MOTOR_ID)
                    );
                } else {
                    intake = null;
                }

                if (Constants.INSTANTIATE_SHOOTER) {
                    shooter = new Shooter(
                        new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID, FuelConstants.SHOOTER_FOLLOWER_ID), 
                        new KickerIOReal(FuelConstants.KICKER_MOTOR_ID)
                    );
                } else {
                    shooter = null;
                }

                if (Constants.INSTANTIATE_LEDS) {
                    ledStrip = new LedStrip();
                } else {
                    ledStrip = null;
                }
                break;
            case SIM: // sim robot, instantiate physics sim IO implementations
                configureSimulation();

                if (Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
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
                            ),
                            new CameraIOPhotonVisionSim(
                                VisionConstants.camera2Name, 
                                VisionConstants.robotToCamera2, 
                                driveSimulation::getSimulatedDriveTrainPose
                            )
                        },
                        drive, 
                        Constants.ROBOT_START_POSE
                    );
                } else {
                    drive = null;
                    poseEstimator = null;
                }

                if (Constants.INSTANTIATE_INTAKE) {
                    intake = new Intake(
                        new IntakeIOSim(), 
                        new PivotIOSim()
                    );
                } else {
                    intake = null;
                }

                if (Constants.INSTANTIATE_SHOOTER) {
                    shooter = new Shooter(
                        new ShooterIOSim(),
                        new KickerIOSim()
                    );
                } else {
                    shooter = null;
                }

                ledStrip = null;

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

                ledStrip = null;
                break;
        }

        if (Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            drive.setPoseEstimator(poseEstimator);
        }

        commandFactory = new CommandFactory(
            controller, 
            drive, 
            poseEstimator, 
            intake, 
            shooter,
            fuelSimulation
        );

        configureButtonBindings();
        configureRobot();
        configureAutos();
    }

    private void configureButtonBindings() {
        if (Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            // drive
            drive.setDefaultCommand(commandFactory.getDriveWithJoysticksCommand());
        }

        // check motors
        SmartDashboard.putData("smartDashboard/buttons/Check Motors", commandFactory.getCheckMotorsCommand());

        switch (Constants.CURRENT_BUTTON_BINDINGS) {
            case COMPETITION: 
                if (Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
                    // x-lock
                    controller.x().whileTrue(
                        Commands.run(() -> drive.xLock())
                    );
                }

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

                // pivot
                controller.leftBumper().onTrue(commandFactory.getTogglePivotCommand());

                controller.b().onTrue(new InstantCommand(
                    () -> {
                        Constants.TRENCH_ALIGN = !Constants.TRENCH_ALIGN;
                        SmartDashboard.putBoolean("smartDashboard/toggles/Trench Align", Constants.TRENCH_ALIGN);
                    }
                ));
                break;
            case TESTING_BPS: 
                controller.leftTrigger().whileTrue(
                    new StartEndCommand(
                        () -> {
                            shooter.setKickerVelocity(RotationsPerSecond.of(kickerVelocity));
                        }, 
                        () -> {
                            shooter.setKickerVelocity(RotationsPerSecond.of(0));
                        }
                    )
                );

                controller.rightTrigger().whileTrue(
                    new StartEndCommand(
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                        }, 
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(0));
                        }
                    )
                );

                // increment shooter and kicker velocity
                controller.x().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity += 1;
                    })
                );
                controller.b().onTrue(
                    new InstantCommand(() -> {
                        shooterVelocity -= 1;
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
                        true
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
            SmartDashboard.putData("smartDashboard/buttons/Clear Fuel", new InstantCommand(() -> fuelSimulation.clearFuel()));
            controller.b().onTrue(new InstantCommand(() -> fuelSimulation.clearFuel()));
        }
    }

    // ————— robot ————— //

    public void configureRobot() {
        SmartDashboard.putBoolean("smartDashboard/toggles/Trench Align", Constants.TRENCH_ALIGN);
        SmartDashboard.putBoolean("smartDashboard/toggles/Use Pivot", Constants.USE_PIVOT);
        SmartDashboard.putBoolean("smartDashboard/toggles/Shoot on the Move", Constants.SHOOT_ON_THE_MOVE);
    }

    public void robotPeriodic() {
        Constants.TRENCH_ALIGN = SmartDashboard.getBoolean("smartDashboard/toggles/Trench Align", Constants.TRENCH_ALIGN);
        Constants.USE_PIVOT = SmartDashboard.getBoolean("smartDashboard/toggles/Use Pivot", Constants.USE_PIVOT);
        Constants.SHOOT_ON_THE_MOVE = SmartDashboard.getBoolean("smartDashboard/toggles/Shoot on the Move", Constants.SHOOT_ON_THE_MOVE);

        Logger.recordOutput("outputs/drive/TRENCH_ALIGN", Constants.TRENCH_ALIGN);
        Logger.recordOutput("outputs/fuelIO/intake/USE_PIVOT", Constants.USE_PIVOT);
        Logger.recordOutput("outputs/fuelIO/shooter/SHOOT_ON_THE_MOVE", Constants.SHOOT_ON_THE_MOVE);

        // ! any way to make this more consise (use an array)
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/blue bottom", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[0].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/blue top", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[1].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/red bottom", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[2].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/red top", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[3].getCorners());

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

    // ————— autonomous ————— //

    private void configureAutos() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return;
        }

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
        autoChooser.addRoutine("BeMeanBottom", () -> autoGenerator.beMeanBottom());
        autoChooser.addRoutine("BeMeanTop", () -> autoGenerator.beMeanTop());

        if (Constants.INSTANTIATE_INTAKE
            && Constants.INSTANTIATE_SHOOTER
        ) {
            autoChooser.addCmd("BackUpAndShoot", () -> autoGenerator.backUpAndShoot());
            autoChooser.addRoutine("CenterFuelBottom", () -> autoGenerator.centerFuelBottom());
            autoChooser.addRoutine("CenterFuelTop", () -> autoGenerator.centerFuelTop());
            autoChooser.addRoutine("OutpostFuel", () -> autoGenerator.outpostFuel());

            autoChooser.select("BackUpAndShoot"); // picks a default auto
        }

        SmartDashboard.putData("smartDashboard/AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return new InstantCommand();
        }

        return autoChooser.selectedCommand();
    }

    // ————— teleop ————— //

    public void teleopPeriodic() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL || Constants.REALISTIC_SIM) {
            Logger.recordOutput("outputs/fieldInfo/autoWinner", HubUtil.getAutoWinner());
        }
    }

    // ————— simulation ————— //

    private void configureSimulation() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return;
        }

        // drive
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, Constants.ROBOT_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        Constants.FieldConstants.Zones.logAllZones();

        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return;
        }

        // fuelIO
        fuelSimulation = new FuelSim();
        fuelSimulation.registerRobot(
            DriveConstants.WIDTH_X.in(Meters),
            DriveConstants.WIDTH_Y.in(Meters),
            Inches.of(6).in(Meters),
            () -> poseEstimator.getPose(),
            () -> drive.getFieldRelativeSpeeds()
        );
        fuelSimulation.registerIntake(
            DriveConstants.WIDTH_X.div(2).in(Meters),
            DriveConstants.WIDTH_X.div(2).in(Meters) + FuelConstants.INTAKE_WIDTH_X.in(Meters),
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
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return;
        }

        // drive
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("outputs/simulation/fieldSimulation/robotPosition", driveSimulation.getSimulatedDriveTrainPose());

        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return;
        }

        // fuelIO
        fuelSimulation.stepSim();
    }

    public void resetSimulation() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
        ) {
            return;
        }

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

        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return;
        }

        // fuel
        fuelSimulation.clearFuel();

        if (Constants.REALISTIC_SIM) {
            fuelSimulation.spawnStartingFuel();
        }
    }
}