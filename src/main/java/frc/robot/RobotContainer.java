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
import org.littletonrobotics.junction.Logger;
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

    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    private FuelSim fuelSimulation;

    // ————— testing variables ————— //

    // private double shooterVelocity = 0;
    // private double hoodAngle = 0;

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

        configureButtonBindings();
        configureAutos();
    }

    private void configureButtonBindings() {
        // ————— drive ————— //
        
        // regular joystick drive
        drive.setDefaultCommand(
            new DriveWithJoysticks(
                drive,
                poseEstimator,
                () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped, x velocity relative to field is "forward" from driver perspective
                () -> controller.getLeftXWithDeadband(), 
                () -> controller.getRightXWithDeadband(),
                null
            )
        );

        // ————— processed fuel bindings ————— //

        // intake and turn robot in the direction it's driving
        controller.x().whileTrue(
            new IntakeCommand(
                intake
            ).alongWith(
                new DriveWithJoysticks(
                    drive, 
                    poseEstimator, 
                    () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped
                    () -> controller.getLeftXWithDeadband(), 
                    () -> controller.getRightXWithDeadband(),
                    () -> {
                        return new Rotation2d(Math.atan2( // negatives are to map xbox controller to the cartesian plane
                            -controller.getLeftXWithDeadband(), 
                            -controller.getLeftYWithDeadband()
                        ) + Math.PI); // intake is on back of robot
                    }
                )
            )
        );

        // shoot and turn robot towards target pose
        controller.y().whileTrue(
            new ShootCommand(
                drive,
                poseEstimator,
                intake,
                hopper,
                shooter,
                () -> ShootUtil.getTargetPose(poseEstimator.getPose()) // targetPose
                , true
            ).alongWith(
                new DriveWithJoysticks(
                    drive, 
                    poseEstimator, 
                    () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped
                    () -> controller.getLeftXWithDeadband(), 
                    () -> controller.getRightXWithDeadband(),
                    () -> ShootUtil.getRobotRotationToTarget()
                )
            ).alongWith(
                Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ? 
                new InstantCommand(
                    () -> {
                        if (Constants.REALISTIC_SIM) {
                            if (hopper.getHopperEmpty()) {
                                return;
                            }
                            hopper.shootFuel();
                        }

                        fuelSimulation.launchFuel(
                            () -> shooter.getShooterLinearVelocity(), 
                            () -> shooter.getHoodShotAngle(),
                            Rotations.of(0),
                            FuelConstants.SHOOTER_POSITION
                        );
                    }
                ).andThen(Commands.waitSeconds(0.5)).repeatedly() :
                new InstantCommand()
            )
        );

        controller.b().onTrue(new InstantCommand(() -> fuelSimulation.clearFuel()));

        // ————— raw fuel bindings ————— //

        // intake
        controller.x().onTrue(intake.getSetIntakeVoltageCommand(Volts.of(12)));
        controller.b().onTrue(intake.getSetIntakeVoltageCommand(Volts.of(0)));

        // controller.x().onTrue(intake.getSetPivotPositionCommand 

        // // hopper
        // controller.y().onTrue(hopper.getSetHopperVoltageCommand(Volts.of(5)));
        // controller.a().onTrue(hopper.getSetHopperVoltageCommand(Volts.of(0)));

        // // shooter
        // controller.rightTrigger().onTrue(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)));
        // controller.leftTrigger().onTrue(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(0)));

        // controller.leftBumper().onTrue(shooter.getSetHoodPositionCommand(FuelConstants.HOOD_DOWN_ANGLE));
        // controller.rightBumper().onTrue(shooter.getSetHoodPositionCommand(FuelConstants.HOOD_UP_ANGLE));

        // controller.y().onTrue(shooter.getSetKickerVoltageCommand(Volts.of(3)));
        // controller.a().onTrue(shooter.getSetKickerVoltageCommand(Volts.of(0)));

        // controller.y().onTrue(climber.getSetClimberVoltageCommand(Volts.of(4)));
        // controller.b().onTrue(climber.getSetClimberVoltageCommand(Volts.of(0)));
        // controller.a().onTrue(climber.getSetClimberVoltageCommand(Volts.of(-4)));

        // ————— testing ————— //
        
        // sysid
        // controller.x().whileTrue(drive.sysIdFull());
        // controller.y().whileTrue(Commands.runOnce(SignalLogger::start).andThen(drive.sysIdFull()));
        // controller.a().onFalse(Commands.runOnce(SignalLogger::stop));

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
        //         hoodAngle += 0.01;
        //         shooter.setHoodPosition(Rotations.of(hoodAngle));
        //     })
        // );

        // controller.rightBumper().onTrue(
        //     new InstantCommand(() -> {
        //         hoodAngle -= 0.01;
        //         shooter.setHoodPosition(Rotations.of(hoodAngle));
        //     })
        // );
    }

    // ————— autonomous ————— //

    private void configureAutos() {
        autoGenerator = new AutoGenerator(drive, poseEstimator, driveSimulation, intake, hopper, shooter);
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());

        autoChooser.select("Back Up"); // picks a default auto

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void autonomousPeriodic() {
        if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL || Constants.REALISTIC_SIM) {
            Logger.recordOutput("outputs/simulation/fuelSimulation/remainingShiftTime", HubUtil.timeRemainingInCurrentShift().orElse(Seconds.of(-1)));
            Logger.recordOutput("outputs/simulation/fuelSimulation/currentShift", HubUtil.getCurrentShift().orElse(HubUtil.Shift.NO_SHIFT));
            Logger.recordOutput("outputs/simulation/fuelSimulation/hubActive", HubUtil.isActive());
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
            Logger.recordOutput("outputs/simulation/fuelSimulation/remainingShiftTime", HubUtil.timeRemainingInCurrentShift().orElse(Seconds.of(-1)));
            Logger.recordOutput("outputs/simulation/fuelSimulation/currentShift", HubUtil.getCurrentShift().orElse(HubUtil.Shift.NO_SHIFT));
            Logger.recordOutput("outputs/simulation/fuelSimulation/hubActive", HubUtil.isActive());
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