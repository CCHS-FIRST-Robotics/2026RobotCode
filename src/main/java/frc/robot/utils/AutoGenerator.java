package frc.robot.utils;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import choreo.auto.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.PoseEstimator;

public class AutoGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    public AutoGenerator(
        Drive drive, 
        PoseEstimator poseEstimator,
        SwerveDriveSimulation driveSimulation
    ) {
        autoFactory = new AutoFactory(
            poseEstimator::getPose,
            (pose) -> {
                poseEstimator.resetPosition(pose);
                if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM){
                    driveSimulation.setSimulationWorldPose(pose);
                }
            },
            drive::runAutoPosition,
            Constants.USE_ALLIANCE_FLIPPING, 
            drive
        );

        this.drive = drive;
        this.poseEstimator = poseEstimator;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);
        AutoTrajectory trajectory1 = routine.trajectory("Test", 1);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            // new DriveWithPosition(drive, poseEstimator, trajectory0.getInitialPose().get()) // ! add the catch later
            // .andThen(trajectory0.resetOdometry())
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
            .andThen(trajectory1.cmd())
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUp() {
        return new DriveWithPosition(drive, poseEstimator, new Transform2d(0, 3, new Rotation2d()));
    }
}