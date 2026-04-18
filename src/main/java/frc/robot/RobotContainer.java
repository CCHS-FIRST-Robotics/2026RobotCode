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

    private final Controller controller = new Controller(Constants.CONTROLLER_PORT);

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

    @AutoLogOutput(key = "outputs/testing/thetaPIDPositionTolerance")
    private double thetaPIDPositionTolerance = 0.15;
    @AutoLogOutput(key = "outputs/testing/shooterVelocity")
    private double shooterVelocity = 0;
    @AutoLogOutput(key = "outputs/testing/kickerVelocity")
    private double kickerVelocity = 0;
    @AutoLogOutput(key = "outputs/testing/shooterMapOffset")
    private double shooterMapOffset = 0;

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
                            new CameraIOPhotonVision(VisionConstants.camera1Name, VisionConstants.robotToCamera1)
                        }, 
                        drive, 
                        Constants.ROBOT_START_POSE
                    );
                } else {
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
                }

                if (Constants.INSTANTIATE_INTAKE) {
                    intake = new Intake(
                        new IntakeIOReal(FuelConstants.INTAKE_MOTOR_ID), 
                        new PivotIOReal(FuelConstants.PIVOT_MOTOR_ID)
                    );
                } else {
                    intake = new Intake(
                        new IntakeIO() {}, 
                        new PivotIO() {}
                    );
                }

                if (Constants.INSTANTIATE_SHOOTER) {
                    shooter = new Shooter(
                        new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID, FuelConstants.SHOOTER_FOLLOWER_ID), 
                        new KickerIOReal(FuelConstants.KICKER_MOTOR_ID)
                    );
                } else {
                    shooter = new Shooter(
                        new ShooterIO() {}, 
                        new KickerIO() {}
                    );
                }

                ledStrip = new LedStrip();
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
                                driveSimulation::getSimulatedDriveTrainPose
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
                } else {
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
                }

                if (Constants.INSTANTIATE_INTAKE) {
                    intake = new Intake(
                        new IntakeIOSim(), 
                        new PivotIOSim()
                    );
                } else {
                    intake = new Intake(
                        new IntakeIO() {}, 
                        new PivotIO() {}
                    );
                }

                if (Constants.INSTANTIATE_SHOOTER) {
                    shooter = new Shooter(
                        new ShooterIOSim(),
                        new KickerIOSim()
                    );
                } else {
                    shooter = new Shooter(
                        new ShooterIO() {}, 
                        new KickerIO() {}
                    );
                }

                ledStrip = new LedStrip();

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

                ledStrip = new LedStrip();
                break;
        }

        drive.setPoseEstimator(poseEstimator);

        commandFactory = new CommandFactory(
            controller, 
            drive, 
            poseEstimator, 
            intake, 
            shooter,
            ledStrip,
            fuelSimulation
        );

        configureButtonBindings();
        configureRobot();
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

                // drive slow
                controller.leftStick().whileTrue( // remapped as gamesir L4
                    commandFactory.getDriveSpeedCommand(
                        MetersPerSecond.of(1), 
                        RadiansPerSecond.of(1 / DriveConstants.TRACK_RADIUS), 
                        DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
                        DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
                    )
                );

                // drive fast
                controller.rightStick().whileTrue( // remapped as gamesir R4
                    commandFactory.getDriveSpeedCommand(
                        DriveConstants.MAX_THEORETICAL_LINEAR_SPEED, 
                        DriveConstants.MAX_THEORETICAL_ANGULAR_SPEED, 
                        MetersPerSecondPerSecond.of(100), 
                        RadiansPerSecondPerSecond.of(100)
                    )
                );

                // intake
                controller.leftTrigger().and(controller.rightTrigger().negate()).whileTrue(
                    commandFactory.getDriveSpeedCommand(
                        MetersPerSecond.of(2), 
                        DriveConstants.MAX_ALLOWED_ANGULAR_SPEED, 
                        DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
                        DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
                    )
                    .alongWith(commandFactory.getIntakeCommand())
                    .alongWith(commandFactory.getSetLedStripHuesCommand(new Integer[] {60})) // leds are green
                );

                // drive and intake and shoot
                controller.leftTrigger().and(controller.rightTrigger()).whileTrue(
                    commandFactory.getDriveAndIntakeAndShootCommand()
                    .alongWith(commandFactory.getSetLedStripHuesCommand(new Integer[] {60, 120})) // leds are green and blue
                );

                // drive and shoot
                controller.leftTrigger().negate().and(controller.rightTrigger()).whileTrue(
                    commandFactory.getDriveAndShootCommand(true, true)
                    .alongWith(commandFactory.getShootingLedStripCommand()) // leds are blue or red
                );

                // shoot
                controller.pov(0).whileTrue(commandFactory.getShootCommand(
                    () -> RotationsPerSecond.of(50), 
                    true, 
                    true, 
                    true)
                );
                
                // pivot
                controller.leftBumper().onTrue(commandFactory.getTogglePivotCommand());
                controller.y().onTrue(commandFactory.getSetPivotEncoderPositionUpCommand());
                controller.a().onTrue(commandFactory.getSetPivotEncoderPositionDownCommand());

                // unstick
                controller.pov(180).whileTrue(
                    new StartEndCommand(
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(-10)); // negative for reverse
                            shooter.setKickerVelocity(RotationsPerSecond.of(-10));
                        },
                        () -> {
                            shooter.setShooterVelocity(RotationsPerSecond.of(0));
                            shooter.setKickerVelocity(RotationsPerSecond.of(0));
                        }
                    )
                );

                controller.b().onTrue(new InstantCommand(
                    () -> {
                        Constants.ENABLE_TRENCH_ALIGN = !Constants.ENABLE_TRENCH_ALIGN;
                        SmartDashboard.putBoolean("smartDashboard/toggles/Enable Trench Align", Constants.ENABLE_TRENCH_ALIGN);
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
                        true,
                        true,
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
                        shooterVelocity -= 0.5;
                        shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                    })
                );
                controller.rightTrigger().onTrue(
                    new InstantCommand(() -> { 
                        shooterVelocity += 0.5;
                        shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
                    })
                );

                SmartDashboard.putData("smartDashboard/buttons/Increment Shooter Map", new InstantCommand(
                    () -> {
                        shooterMapOffset++;
                        ShootUtil.offsetShooterMap(shooterMapOffset);
                    }
                ));
                SmartDashboard.putData("smartDashboard/buttons/Decrement Shooter Map", new InstantCommand(
                    () -> {
                        shooterMapOffset--;
                        ShootUtil.offsetShooterMap(shooterMapOffset);
                    }
                ));
                break;
        }

        // ————— simulation bindings ————— //

        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) {
            SmartDashboard.putData("smartDashboard/buttons/Clear Fuel", new InstantCommand(() -> fuelSimulation.clearFuel()));
            controller.b().onTrue(new InstantCommand(() -> fuelSimulation.clearFuel()));
        }

        // ————— testing bindings ————— //

        SmartDashboard.putData("smartDashboard/buttons/Check Motors", commandFactory.getCheckMotorsCommand());

        // change theta PID position tolerance
        SmartDashboard.putData("smartDashboard/buttons/Increase Theta PID Position Tolerance", new InstantCommand(
            () -> {
                thetaPIDPositionTolerance += 0.1;
                drive.setThetaPIDPositionTolerance(thetaPIDPositionTolerance);
            }
        ));
        SmartDashboard.putData("smartDashboard/buttons/Decrease Theta PID Position Tolerance", new InstantCommand(
            () -> {
                thetaPIDPositionTolerance -= 0.1;
                drive.setThetaPIDPositionTolerance(thetaPIDPositionTolerance);
            }
        ));
    }

    // ————— robot ————— //

    public void configureRobot() {
        SmartDashboard.putBoolean("smartDashboard/toggles/Enable Trench Align", Constants.ENABLE_TRENCH_ALIGN);
        SmartDashboard.putBoolean("smartDashboard/toggles/Enable Pivot", Constants.ENABLE_PIVOT);
        SmartDashboard.putBoolean("smartDashboard/toggles/Enable Pivot Agitation", Constants.ENABLE_PIVOT_AGITATION);
        SmartDashboard.putBoolean("smartDashboard/toggles/Enable Shoot on the Move", Constants.ENABLE_SHOOT_ON_THE_MOVE);
    }

    public void robotPeriodic() {
        Constants.ENABLE_TRENCH_ALIGN = SmartDashboard.getBoolean("smartDashboard/toggles/Enable Trench Align", Constants.ENABLE_TRENCH_ALIGN);
        Constants.ENABLE_PIVOT = SmartDashboard.getBoolean("smartDashboard/toggles/Enable Pivot", Constants.ENABLE_PIVOT);
        Constants.ENABLE_PIVOT_AGITATION = SmartDashboard.getBoolean("smartDashboard/toggles/Enable Pivot Agitation", Constants.ENABLE_PIVOT_AGITATION);
        Constants.ENABLE_SHOOT_ON_THE_MOVE = SmartDashboard.getBoolean("smartDashboard/toggles/Enable Shoot on the Move", Constants.ENABLE_SHOOT_ON_THE_MOVE);

        Logger.recordOutput("outputs/drive/ENABLE_TRENCH_ALIGN", Constants.ENABLE_TRENCH_ALIGN);
        Logger.recordOutput("outputs/fuelIO/intake/ENABLE_PIVOT", Constants.ENABLE_PIVOT);
        Logger.recordOutput("outputs/fuelIO/intake/ENABLE_PIVOT_AGITATION", Constants.ENABLE_PIVOT_AGITATION);
        Logger.recordOutput("outputs/fuelIO/shooter/ENABLE_SHOOT_ON_THE_MOVE", Constants.ENABLE_SHOOT_ON_THE_MOVE);

        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/current/blue left", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[0].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/current/blue right", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[1].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/current/red left", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[2].getCorners());
        Logger.recordOutput("outputs/simulation/fieldSimulation/zones/trenches/current/red right", Constants.FieldConstants.Zones.TRENCH_ZONES.zones[3].getCorners());

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
        autoGenerator = new AutoGenerator(
            drive, 
            poseEstimator, 
            intake, 
            shooter, 
            driveSimulation, 
            commandFactory
        );
        autoChooser = new AutoChooser(); // creates and selects a "do nothing" auto by default

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addRoutine("BeMeanLeft", () -> autoGenerator.beMeanLeft());
        autoChooser.addRoutine("BeMeanRight", () -> autoGenerator.beMeanRight());

        if (Constants.INSTANTIATE_INTAKE
            && Constants.INSTANTIATE_SHOOTER
        ) {
            autoChooser.addCmd("BackUpAndShoot", () -> autoGenerator.backUpAndShoot());
            autoChooser.addRoutine("CenterFuelLeft ", () -> autoGenerator.centerFuelLeft());
            autoChooser.addRoutine("CenterFuelRight", () -> autoGenerator.centerFuelRight());
            autoChooser.addRoutine("RepeatingCenterFuelLeft", () -> autoGenerator.repeatingCenterFuelLeft());
            autoChooser.addRoutine("RepeatingCenterFuelRight", () -> autoGenerator.repeatingCenterFuelRight());
            autoChooser.addRoutine("RepeatingCenterFuelLeftCloser", () -> autoGenerator.repeatingCenterFuelLeftCloser());
            autoChooser.addRoutine("RepeatingCenterFuelRightCloser", () -> autoGenerator.repeatingCenterFuelRightCloser());
            autoChooser.addRoutine("OutpostFuel", () -> autoGenerator.outpostFuel());

            autoChooser.select("BackUpAndShoot"); // picks a default auto
        }

        SmartDashboard.putData("smartDashboard/AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() {
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
        // drive
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, Constants.ROBOT_START_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        Constants.FieldConstants.Zones.logAllZones();

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
        // drive
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("outputs/simulation/fieldSimulation/robotPosition", driveSimulation.getSimulatedDriveTrainPose());

        // fuelIO
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