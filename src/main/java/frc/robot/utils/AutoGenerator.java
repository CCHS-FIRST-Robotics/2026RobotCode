package frc.robot.utils;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import choreo.auto.*;
import org.ironmaple.simulation.drivesims.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.Constants;

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
                if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM) {
                    driveSimulation.setSimulationWorldPose(pose);
                }
            },
            drive::runAutoPosition,
            true, // ! I fucking think (idk what "If this returns true, when on the red alliance" is supposed to mean)
            drive
        );
        choreo.util.ChoreoAllianceFlipUtil.flip(new Pose2d());

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
        return new DriveWithPosition(drive, poseEstimator, new Transform2d(-2, 0, new Rotation2d()));
    }
}