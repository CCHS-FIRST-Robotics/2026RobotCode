package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.math.geometry.*;
import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.subsystems.poseEstimator.odometry.*;
import frc.robot.subsystems.poseEstimator.vision.*;
import frc.robot.utils.*;

public class RobotContainer {
    // controllers
    private final CommandXboxController controller = new CommandXboxController(0);

    // subsystems
    private final Drive drive;
    private final PoseEstimator poseEstimator;

    // utils
    private AutoGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;
    
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

        // controller.b().whileTrue(
        //     new DriveWithPosition(
        //         drive, 
        //         poseEstimator, 
        //         new Pose2d(1, 3.5, new Rotation2d(0))
        //     )
        // );
        // controller.x().whileTrue(
        //     new DriveWithPosition(
        //         drive, 
        //         poseEstimator, 
        //         new Pose2d(2, 3.5, new Rotation2d(0))
        //     )
        // );

        // controller.x().whileTrue(drive.sysIdFull());
        // controller.y().whileTrue(Commands.runOnce(SignalLogger::start).andThen(drive.sysIdFull()));
        // controller.a().onFalse(Commands.runOnce(SignalLogger::stop));

        controller.x().onTrue(
            new InstantCommand(
                () -> {
                    NoteOnFly noteOnFly = new NoteOnFly(
                        // Specify the position of the chassis when the note is launched
                        driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                        // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
                        new Translation2d(),
                        // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
                        drive.getPrevSpeeds(),
                        // The shooter facing direction is the same as the robot’s facing direction
                        driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                        // Initial height of the flying note
                        Meters.of(0.45),
                        // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
                        MetersPerSecond.of(5),
                        // The angle at which the note is launched (45 degrees for a visible arc)
                        Radians.of(Math.PI / 4)
                    );
                    SimulatedArena.getInstance().addGamePieceProjectile(noteOnFly);
                }
            )
        );
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
        driveSimulation = new SwerveDriveSimulation(DriveConstants.DRIVE_SIMULATION_CONFIG, startPose);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // SimulatedArena.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(Math.random() * 10, Math.random() * 10, new Rotation2d())));
    }

    public void updateSimulation() { // called by Robot.java on simulationPeriodic
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) { // ! not sure if this has to be here if it's only called in simulationPeriodic
            return;
        }

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
    }

    public void resetSimulationField() { // called by Robot.java on disabledInit (only runs if in SIM mode)
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) {
            return;
        }

        driveSimulation.setSimulationWorldPose(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}