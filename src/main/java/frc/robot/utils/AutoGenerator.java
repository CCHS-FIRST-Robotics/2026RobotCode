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
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class AutoGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Shooter shooter;

    private final CommandFactory commandFactory;

    public AutoGenerator(
        Drive drive, 
        PoseEstimator poseEstimator,
        Intake intake,
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
            drive::runPositionChoreo,
            true,
            drive
        );
        choreo.util.ChoreoAllianceFlipUtil.flip(new Pose2d());

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.intake = intake;
        this.shooter = shooter;

        this.commandFactory = commandFactory;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUpAndShoot() {
        return commandFactory.getSetPivotDownCommand()
        .alongWith(new DriveWithPosition(drive, poseEstimator, new Transform2d(-1.5, 0, new Rotation2d())))
        .andThen(commandFactory.getDriveAndShootCommand(true, true));
    }

    public AutoRoutine beMeanLeft() {
        AutoRoutine routine = autoFactory.newRoutine("BeMeanLeft");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("BeMeanLeft", 0);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    public AutoRoutine beMeanRight() {
        AutoRoutine routine = autoFactory.newRoutine("BeMeanRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("BeMeanRight", 0);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    public AutoRoutine centerFuelLeft() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelLeft");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelLeft", 0); // bring pivot down
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelLeft", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelLeft", 2); // stop intake
        // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(
                    Commands.waitSeconds(0.5) // time it takes for robot to not be under trench anymore
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(Commands.waitSeconds(0.5)) // wait while pivot is coming down
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true, true))
        );

        return routine;
    }

    public AutoRoutine centerFuelRight() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelRight", 0); // bring pivot down
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelRight", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelRight", 2); // stop intake, spin up shooter
        // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(
                    Commands.waitSeconds(0.5)
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true, true))
        );

        return routine;
    }

    public AutoRoutine repeatingCenterFuelRight() {
        AutoRoutine routine = autoFactory.newRoutine("RepeatingCenterFuelRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("RepeatingCenterFuelRight", 0); // bring pivot down
        AutoTrajectory trajectory1 = routine.trajectory("RepeatingCenterFuelRight", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("RepeatingCenterFuelRight", 2); // stop intake, spin up shooter
        // shoot
        AutoTrajectory trajectory4 = routine.trajectory("RepeatingCenterFuelRight", 4); // 

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(
                    Commands.waitSeconds(0.5)
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true, true).withTimeout(4))
            .andThen(trajectory4.cmd())
            .repeatedly()
        );

        return routine;
    }

    public AutoRoutine outpostFuel() {
        AutoRoutine routine = autoFactory.newRoutine("OutpostFuel");

        AutoTrajectory trajectory0 = routine.trajectory("OutpostFuel", 0);
        // wait for outpost fuel
        AutoTrajectory trajectory1 = routine.trajectory("OutpostFuel", 1);
        // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(commandFactory.getSetPivotDownCommand())
            )
            .andThen(trajectory0.cmd())
            .andThen(Commands.waitSeconds(4))
            .andThen(trajectory1.cmd())
            .andThen(commandFactory.getDriveAndShootCommand(true, true))
        );

        return routine;
    }
}