package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import choreo.auto.AutoChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.*;

public class RobotContainer {
    // controllers
    private final CommandXboxController controller = new CommandXboxController(0);

    // subsystems
    private final Drive drive;
    private final Vision vision; // ! make this poseEstimator

    // utils
    private AutoRoutineGenerator autoGenerator;
    private AutoChooser autoChooser;
    private SwerveDriveSimulation driveSimulation;

    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL: // real robot, instantiate hardware IO implementations
                drive = new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIOTalonFXReal(DriveConstants.FrontLeft),
                    new ModuleIOTalonFXReal(DriveConstants.FrontRight),
                    new ModuleIOTalonFXReal(DriveConstants.BackLeft),
                    new ModuleIOTalonFXReal(DriveConstants.BackRight),
                    (pose) -> {} // ! hmm
                );
                this.vision = new Vision(
                    drive,
                    new VisionIOPhotonVision(VisionConstants.camera0Name, new Transform3d()),
                    new VisionIOPhotonVision(VisionConstants.camera1Name, new Transform3d())
                );
                break;
            case SIM: // sim robot, instantiate physics sim IO implementations
                configureSimulation();

                drive = new Drive(
                    new GyroIOSim(driveSimulation.getGyroSimulation()),
                    new ModuleIOTalonFXSim(DriveConstants.FrontLeft, driveSimulation.getModules()[0]),
                    new ModuleIOTalonFXSim(DriveConstants.FrontRight, driveSimulation.getModules()[1]),
                    new ModuleIOTalonFXSim(DriveConstants.BackLeft, driveSimulation.getModules()[2]),
                    new ModuleIOTalonFXSim(DriveConstants.BackRight, driveSimulation.getModules()[3]),
                    driveSimulation::setSimulationWorldPose
                );
                vision = new Vision(
                    drive,
                    new VisionIOPhotonVisionSim(
                        VisionConstants.camera0Name, 
                        VisionConstants.robotToCamera0, 
                        driveSimulation::getSimulatedDriveTrainPose
                    ),
                    new VisionIOPhotonVisionSim(
                        VisionConstants.camera1Name, 
                        VisionConstants.robotToCamera1, 
                        driveSimulation::getSimulatedDriveTrainPose
                    )
                );
                break;
            default: // replayed robot, disable IO implementations
                drive = new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    (pose) -> {}
                );
                vision = new Vision(
                    drive, 
                    new VisionIO() {}, 
                    new VisionIO() {}
                );
                break;
        }

        configureButtonBindings();
        configureAutos();
    }

    private void configureButtonBindings() {
        // ————— drive ————— //
        
        // regular joystick drive
        drive.setDefaultCommand(
            new DriveWithVelocity(
                drive, 
                () -> -controller.getLeftY(), // xbox controller is flipped
                () -> -controller.getLeftX(), // ! not sure why
                () -> controller.getRightX()
            )
        );

        // x-lock
        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // testing
        controller.a().whileTrue(new DriveWithPosition(drive, new Pose2d(1, 5, new Rotation2d(Math.PI/2))));

        // ————— uhhhhhh ————— //

        // ! ermmmmmmmmm idk: 
        // final Runnable resetGyro = 
        // Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ? 
        // () -> drive.setPose(
        //     driveSimulation.getSimulatedDriveTrainPose()
        // ) // reset odometry to actual robot pose during simulation // ! not sure what this means
        // : () -> drive.setPose(
        //     new Pose2d(drive.getPose().getTranslation(), new Rotation2d()) // zero gyro
        // );
        
        // ! idk why it has to be run by the controller (wait, does xbox controller have a literal start button)
        // controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    // ————— autos ————— //

    private void configureAutos() {
        autoGenerator = new AutoRoutineGenerator(drive);
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Test", () -> autoGenerator.test());
        autoChooser.addCmd("Back Up", () -> autoGenerator.backUp());

        autoChooser.select("Back Up"); // pick a default auto

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public Command getAutonomousCommand() { // called by Robot.java on autonomousInit
        return new InstantCommand();
    }

    // ————— simulation ————— //

    private void configureSimulation() {
        driveSimulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
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

        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}