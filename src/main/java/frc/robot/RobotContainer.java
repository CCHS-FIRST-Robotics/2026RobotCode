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

@SuppressWarnings("unused")
public class RobotContainer {
    // ————— controllers ————— //

    private final Controller controller = new Controller(Constants.CONTROLLER_PORT);

    // ————— subsystems ————— //

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;
    private final Climber climber;

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
                hopper = new Hopper(new HopperIOReal(FuelConstants.HOPPER_MOTOR_ID));
                shooter = new Shooter(
                    new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID), 
                    new HoodIOReal(FuelConstants.HOOD_MOTOR_ID), 
                    new KickerIOReal(FuelConstants.KICKER_MOTOR_ID)
                );
                climber = new Climber(new ClimberIOReal(FuelConstants.CLIMBER_MOTOR_ID));
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
                hopper = new Hopper(new HopperIOSim());
                shooter = new Shooter(
                    new ShooterIOSim(),
                    new HoodIOSim(),
                    new KickerIOSim()
                );
                climber = new Climber(new ClimberIO() {});
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
                hopper = new Hopper(new HopperIO() {});
                shooter = new Shooter(
                    new ShooterIO() {}, 
                    new HoodIO() {},
                    new KickerIO() {}
                );
                climber = new Climber(new ClimberIO() {});
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

    private void configureButtonBindings() {
        // ————— competition bindings ————— //

        // drive
        drive.setDefaultCommand(commandFactory.getDriveWithJoysticksCommand());

        // x-lock
        controller.x().whileTrue(
            Commands.run(() -> drive.runCharacterization(
                new Voltage[] {Volts.of(0), Volts.of(0), Volts.of(0), Volts.of(0)}, 
                new Angle[] {Rotations.of(0.125), Rotations.of(0.325), Rotations.of(0.325), Rotations.of(0.125)})
            )
        );

        // drive slow
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

        // ————— simulation bindings ————— //

        // controller.b().onTrue(new InstantCommand(() -> fuelSimulation.clearFuel()));

        // ————— testing for pivot ————— //

        // controller.leftTrigger().onTrue(
        //     new InstantCommand(() -> {
        //         hopperVelocity -= 0.5;
        //         intake.setPivotVoltage(Volts.of(hopperVelocity));
        //     })
        // );
        // controller.rightTrigger().onTrue(
        //     new InstantCommand(() -> {
        //         hopperVelocity += 0.5;
        //         intake.setPivotVoltage(Volts.of(hopperVelocity));
        //     })
        // );

        // controller.leftBumper().onTrue(intake.getSetPivotPositionCommand(Rotations.of(0.381333)));
        // controller.y().onTrue(intake.getSetPivotPositionCommand(Rotations.of(0.2)));
        // controller.b().onTrue(intake.getSetPivotPositionCommand(Rotations.of(0)));
        // controller.a().onTrue(intake.getSetPivotPositionCommand(Rotations.of(-0.04)));

        // ————— testing for position control / autos ————— //

        // controller.x().whileTrue(
        //     new DriveWithPosition(
        //         drive, 
        //         poseEstimator, 
        //         new Pose2d(1.5, 2.5, new Rotation2d())
        //     )
        // );

        // ————— testing for climb ————— //

        // controller.y().onTrue(climber.getSetClimberVoltageCommand(Volts.of(2)));
        // controller.b().onTrue(climber.getSetClimberVoltageCommand(Volts.of(0)));
        // controller.a().onTrue(climber.getSetClimberVoltageCommand(Volts.of(-2)));

        // ————— testing for BPS ————— //

        // // ! remember to comment out the set hopper and kicker voltage in commandfactory

        // controller.rightTrigger().whileTrue(commandFactory.getShootCommand(
        //     () -> new ShooterState(
        //         RotationsPerSecond.of(30), 
        //         Rotations.of(0)
        //     ), 
        //     () -> false
        // ));

        // controller.x().onTrue(
        //     new InstantCommand(() -> {
        //         hopperVelocity += 5;
        //     })
        // );
        // controller.b().onTrue(
        //     new InstantCommand(() -> {
        //         hopperVelocity -= 5;
        //     })
        // );
        // controller.y().onTrue(
        //     new InstantCommand(() -> {
        //         kickerVelocity += 10;
        //     }) 
        // );
        // controller.a().onTrue(
        //     new InstantCommand(() -> {
        //         kickerVelocity -= 10;
        //     })
        // );

        // controller.leftTrigger().whileTrue(
        //     new StartEndCommand(
        //         () -> {
        //             hopper.setHopperVelocity(RotationsPerSecond.of(hopperVelocity));
        //             shooter.setKickerVelocity(RotationsPerSecond.of(kickerVelocity));
        //         },
        //         () -> {
        //             hopper.setHopperVelocity(RotationsPerSecond.of(0));
        //             shooter.setKickerVelocity(RotationsPerSecond.of(0));
        //         }
        //     )
        // );

        // controller.leftBumper().onTrue(hopper.getSetHopperVelocityCommand(RotationsPerSecond.of(-1)));

        // ————— testing for shooter map ————— //

        // // ! remember to uncomment the hopper and kicker voltage in commandfactory

        // controller.x().whileTrue(
        //     commandFactory.getUpdateShootUtilCommand()
        //     .alongWith(commandFactory.getDriveWithJoysticksShooterCommand())
        // );

        // // ! remember to comment out the finallyDo
        // controller.y().whileTrue(commandFactory.getShootCommand(
        //     () -> new ShooterState(
        //         RotationsPerSecond.of(shooter.shooterIOInputs.velocitySetpoint),
        //         Rotations.of(shooter.hoodIOInputs.positionSetpoint)
        //     ), 
        //     () -> false
        // ));

        // controller.leftTrigger().onTrue(
        //     new InstantCommand(() -> {
        //         shooterVelocity -= 1.25;
        //         shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
        //     })
        // );

        // controller.rightTrigger().onTrue(
        //     new InstantCommand(() -> {
        //         shooterVelocity += 1.25;
        //         shooter.setShooterVelocity(RotationsPerSecond.of(shooterVelocity));
        //     })
        // );

        // controller.leftBumper().onTrue(
        //     new InstantCommand(() -> {
        //         hoodAngle -= 0.01;
        //         shooter.setHoodPosition(Rotations.of(hoodAngle));
        //     })
        // );

        // controller.rightBumper().onTrue(
        //     new InstantCommand(() -> {
        //         hoodAngle += 0.01;
        //         shooter.setHoodPosition(Rotations.of(hoodAngle));
        //     })
        // );

        // controller.a().onTrue(
        //     new InstantCommand(() -> 
        //         {
        //             System.out.println("SHOOTER_STATE_MAP.put(DISTANCE, new ShooterState(RotationsPerSecond.of(" + shooter.shooterIOInputs.velocity + "), Rotations.of(" + shooter.hoodIOInputs.position +  ")));\n");
        //         }
        //     )
        // );
    }

    // ————— autonomous ————— //

    private void configureAutos() {
        autoGenerator = new AutoGenerator(
            drive, 
            poseEstimator, 
            intake, 
            hopper, 
            shooter, 
            driveSimulation, 
            commandFactory
        );
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());
        autoChooser.addRoutine("CenterFuel", () -> autoGenerator.centerFuel());
        autoChooser.addRoutine("OutpostFuel", () -> autoGenerator.outpostFuel());

        autoChooser.select("Back Up"); // picks a default auto

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
                return intake.getIntakeOn() && (Constants.REALISTIC_SIM ? !hopper.getHopperFull() : true);
            }, 
            () -> {
                if (Constants.REALISTIC_SIM) {
                    if (hopper.getHopperFull()) {
                        return;
                    }
                    hopper.intakeFuel();
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