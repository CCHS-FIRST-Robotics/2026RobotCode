package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.math.geometry.*;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;
import frc.robot.utils.*;

public class RobotContainer {
    // controllers
    private final CommandXboxController controller = new CommandXboxController(Constants.CONTROLLER_PORT);

    // subsystems
    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    // utils
    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    private FuelSim fuelSimulation;

    private final Pose2d startPose = Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM
        ? new Pose2d(3, 3, new Rotation2d())
        : new Pose2d(0, 0, new Rotation2d());

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
                    startPose
                );
                intake = new Intake(
                    new IntakeIOReal(FuelConstants.INTAKE_MOTOR_ID), 
                    new PivotIOReal(FuelConstants.PIVOT_MOTOR_ID)
                );
                hopper = new Hopper(new HopperIOReal(FuelConstants.HOPPER_MOTOR_ID));
                shooter = new Shooter(
                    new ShooterIOReal(FuelConstants.SHOOTER_MOTOR_ID), 
                    new AnglerIOReal(FuelConstants.ANGLER_MOTOR_ID), 
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
                    startPose
                );
                intake = new Intake(
                    new IntakeIOSim(), 
                    new PivotIOSim()
                );
                hopper = new Hopper(new HopperIOSim());
                shooter = new Shooter(
                    new ShooterIOSim(),
                    new AnglerIOSim(),
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
                hopper = new Hopper(new HopperIO() {});
                shooter = new Shooter(
                    new ShooterIO() {}, 
                    new AnglerIO() {},
                    new KickerIO() {}
                );
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
                () -> -controller.getLeftY(), // xbox controller is flipped
                () -> controller.getLeftX(), 
                () -> controller.getRightX()
            )
        );

        // controller.x().onTrue(new InstantCommand(() -> drive.toggleFollowIntake())); // ! test ! maybe for choreo purposes just ignore the choreo angle?

        // ————— fuel ————— //

        // intake
        // controller.x().onTrue(intake.getSetIntakeVoltageCommand(Volts.of(12)));
        // controller.b().onTrue(intake.getSetIntakeVoltageCommand(Volts.of(0)));

        // controller.x().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_UP_ANGLE));
        // controller.y().onTrue(intake.getSetPivotPositionCommand(Rotations.of(0.1)));
        // controller.b().onTrue(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_DOWN_ANGLE));

        // hopper
        // controller.x().onTrue(hopper.getSetHopperVoltageCommand(Volts.of(5)));
        // controller.b().onTrue(hopper.getSetHopperVoltageCommand(Volts.of(0)));
        
        // shooter
        controller.x().onTrue(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(20)));
        controller.y().onTrue(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(30)));
        controller.b().onTrue(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(50)));
        controller.a().onTrue(shooter.getSetShooterVoltageCommand(Volts.of(0)));

        // controller.x().onTrue(shooter.getSetAnglerPositionCommand(FuelConstants.ANGLER_UP_ANGLE));
        // controller.y().onTrue(shooter.getSetAnglerPositionCommand(Rotations.of(0.12)));
        // controller.b().onTrue(shooter.getSetAnglerPositionCommand(FuelConstants.ANGLER_DOWN_ANGLE));
        // controller.x().onTrue(shooter.getSetAnglerPositionCommand(Rotations.of(0)));
        // controller.b().onTrue(shooter.getSetAnglerPositionCommand(Rotations.of(0.05)));

        // controller.y().onTrue(shooter.getSetKickerVoltageCommand(Volts.of(5)));
        // controller.a().onTrue(shooter.getSetKickerVoltageCommand(Volts.of(0)));

        // ————— misc. testing ————— //

        // fuelSimulation
        // controller.x().onTrue(new InstantCommand(() -> fuelSimulation.launchFuel(
        //     () -> shooter.getShooterLinearVelocity(), 
        //     () -> shooter.getAnglerAngle(),
        //     Rotations.of(0),
        //     new Transform3d(Inches.of(11), Inches.of(0), Inches.of(18), new Rotation3d())
        // ))); 

        // sysid
        // controller.x().whileTrue(drive.sysIdFull());
        // controller.y().whileTrue(Commands.runOnce(SignalLogger::start).andThen(drive.sysIdFull()));
        // controller.a().onFalse(Commands.runOnce(SignalLogger::stop));
    }

    // ————— autos ————— //

    private void configureAutos() {
        autoGenerator = new AutoGenerator(drive, poseEstimator, driveSimulation);
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());

        autoChooser.select("Back Up"); // pick a default auto

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() { // called by Robot.java on autonomousInit
        return autoChooser.selectedCommand();
    }

    // ————— simulation ————— //

    private void configureSimulation() {
        // drive
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, startPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // drive
        fuelSimulation = new FuelSim("FuelSimulation");
        fuelSimulation.registerRobot(
            DriveConstants.WIDTH_X.in(Meters),
            DriveConstants.WIDTH_Y.in(Meters),
            Inches.of(6).in(Meters),
            () -> poseEstimator.getPose(),
            () -> drive.getPrevSpeeds() // ! might be robot relative and probably is
        );
        fuelSimulation.registerIntake(
            -DriveConstants.WIDTH_X.div(2).in(Meters) - Inches.of(11.5).in(Meters),
            -DriveConstants.WIDTH_X.div(2).in(Meters),
            -DriveConstants.WIDTH_Y.div(2).in(Meters),
            DriveConstants.WIDTH_Y.div(2).in(Meters),
            () -> intake.getIntakeOn(), 
            () -> hopper.intakeFuel()
        );
        fuelSimulation.spawnStartingFuel();
        fuelSimulation.setSubticks(1);
        fuelSimulation.start();
    }

    public void updateSimulation() { // called by Robot.java on simulationPeriodic
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) { // ! not sure if this has to be here if it's only called in simulationPeriodic
            return;
        }

        // drive
        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral")
        );
        Logger.recordOutput(
            "FieldSimulation/Algae", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae")
        );
        Logger.recordOutput(
            "FieldSimulation/Note", 
            SimulatedArena.getInstance().getGamePiecesArrayByType("Note")
        );

        // fuel
        fuelSimulation.stepSim();
    }

    public void resetSimulationField() { // called by Robot.java on disabledInit (only runs if in SIM mode)
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) {
            return;
        }

        driveSimulation.setSimulationWorldPose(startPose);
        poseEstimator.resetPosition(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}