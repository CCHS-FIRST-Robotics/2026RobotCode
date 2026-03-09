package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import choreo.auto.*;
import org.ironmaple.simulation.drivesims.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.FuelConstants;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class AutoGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    private final CommandFactory commandFactory;

    public AutoGenerator(
        Drive drive, 
        PoseEstimator poseEstimator,
        Intake intake,
        Hopper hopper,
        Shooter shooter,
        SwerveDriveSimulation driveSimulation,
        CommandFactory commandFactory
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
            true,
            drive
        );
        choreo.util.ChoreoAllianceFlipUtil.flip(new Pose2d());

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;

        this.commandFactory = commandFactory;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);
        AutoTrajectory trajectory1 = routine.trajectory("Test", 1);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            // new DriveWithPosition(drive, poseEstimator, trajectory0.getInitialPose().get())
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

    public AutoRoutine intakeAndShoot() {
        AutoRoutine routine = autoFactory.newRoutine("IntakeAndShoot");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("IntakeAndShoot", 0);
        AutoTrajectory trajectory1 = routine.trajectory("IntakeAndShoot", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("IntakeAndShoot", 2); // stop intake
        AutoTrajectory trajectory3 = routine.trajectory("IntakeAndShoot", 3); // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            (
                trajectory0.resetOdometry()
                .alongWith(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE))
            )
            .andThen(trajectory0.cmd())
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(12)))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0)))
            )
            .andThen(trajectory3.cmd())
            .andThen(commandFactory.getDriveAndShootCommand())
        );

        return routine;
    }
}