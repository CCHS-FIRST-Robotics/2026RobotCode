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
            drive::runPositionChoreo,
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

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUpAndShoot() {
        return intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE)
        .alongWith(new DriveWithPosition(drive, poseEstimator, new Transform2d(-1.5, 0, new Rotation2d())))
        .andThen(commandFactory.getDriveAndShootCommand(true));
    }

    public AutoRoutine beMeanBottom() {
        AutoRoutine routine = autoFactory.newRoutine("BeMeanBottom");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("BeMeanBottom", 0);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    public AutoRoutine beMeanTop() {
        AutoRoutine routine = autoFactory.newRoutine("BeMeanTop");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("BeMeanTop", 0);

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
        );

        return routine;
    }

    public AutoRoutine centerFuelBottom() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelBottom");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelBottom", 0); // bring pivot down
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelBottom", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelBottom", 2); // stop intake
        // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(
                    Commands.waitSeconds(1)
                    .andThen(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE))
                )
            )
            .andThen(Commands.waitSeconds(1.5))
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10)))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true))
        );

        return routine;
    }

    public AutoRoutine centerFuelTop() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelTop");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelTop", 0); // bring pivot down
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelTop", 1); // begin intake
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelTop", 2); // stop intake
        // shoot

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(
                trajectory0.cmd()
                .alongWith(
                    Commands.waitSeconds(1)
                    .andThen(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE))
                )
            )
            .andThen(Commands.waitSeconds(1.5))
            .andThen(
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10)))
            )
            .andThen(
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true))
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
                .alongWith(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE))
            )
            .andThen(trajectory0.cmd())
            .andThen(Commands.waitSeconds(4))
            .andThen(trajectory1.cmd())
            .andThen(commandFactory.getDriveAndShootCommand(true))
        );

        return routine;
    }
}