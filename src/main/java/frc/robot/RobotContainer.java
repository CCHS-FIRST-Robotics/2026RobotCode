package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.subsystems.poseEstimator.odometry.GyroIO;
import frc.robot.subsystems.poseEstimator.odometry.GyroIOPigeon2;
import frc.robot.subsystems.poseEstimator.odometry.GyroIOSim;
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
    
    private final Pose2d startPose = new Pose2d(3, 3, new Rotation2d());

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

        // testing
        controller.b().whileTrue(new DriveWithPosition(drive, poseEstimator, new Pose2d(1, 5, new Rotation2d(Math.PI/2))));
        // ! test driving forwards
        controller.a().whileTrue(Commands.run(() -> drive.runVelocity(
            new ChassisSpeeds(
                0.5,
                0,
                0
            )
        )));
        
        // controller.x().whileTrue(drive.sysIdFull());

        controller.x().whileTrue(Commands.runOnce(SignalLogger::start).andThen(drive.sysIdFull()));
        controller.x().onFalse(Commands.runOnce(SignalLogger::stop));
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
    }

    public void resetSimulationField() { // called by Robot.java on disabledInit (only runs if in SIM mode)
        if (Constants.CURRENT_MODE != Constants.ROBOT_MODE.SIM) {
            return;
        }

        driveSimulation.setSimulationWorldPose(startPose);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}