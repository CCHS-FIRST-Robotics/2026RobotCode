/**
 * Original code
 */

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import choreo.Choreo;
import choreo.auto.*;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import org.ironmaple.simulation.drivesims.*;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.Constants;

public class AutoGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Shooter shooter;

    private final CommandFactory commandFactory;

    private final HashMap<String, String> nameFileMap = new HashMap<String, String>(); // ! what's the rule for creating the object inline

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

        this.drive = drive;
        this.poseEstimator = poseEstimator;

        this.intake = intake;
        this.shooter = shooter;

        this.commandFactory = commandFactory;

        nameFileMap.put("Test", "Test");
        nameFileMap.put("BeMeanLeft", "BeMeanLeft");
        nameFileMap.put("BeMeanRight", "BeMeanRight");
        nameFileMap.put("CenterFuelLeft", "CenterFuelLeft");
        nameFileMap.put("CenterFuelRight", "CenterFuelRight");
        nameFileMap.put("RepeatingCenterFuelLeft", "RepeatingCenterFuelLeft1");
        nameFileMap.put("RepeatingCenterFuelRight", "RepeatingCenterFuelRight1");
        nameFileMap.put("RepeatingCenterFuelLeftCloser", "RepeatingCenterFuelLeft1Closer");
        nameFileMap.put("RepeatingCenterFuelRightCloser", "RepeatingCenterFuelRight1Closer");
        nameFileMap.put("OutpostFuel", "OutpostFuel");
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);

        // routine composition
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
        AutoTrajectory trajectory1 = routine.trajectory("BeMeanLeft", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
            .andThen(
                trajectory1.cmd()
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine beMeanRight() {
        AutoRoutine routine = autoFactory.newRoutine("BeMeanRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("BeMeanRight", 0);
        AutoTrajectory trajectory1 = routine.trajectory("BeMeanRight", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen(trajectory0.cmd())
            .andThen(
                trajectory1.cmd()
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine centerFuelLeft() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelLeft");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelLeft", 0);
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelLeft", 1);
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelLeft", 2);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
            .andThen( // drive and intake
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
            )
            .andThen( // drive and stop intake and spin up shooter
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true, true)) // shoot
        );

        return routine;
    }

    public AutoRoutine centerFuelRight() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelRight", 0);
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelRight", 1);
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelRight", 2);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
            .andThen( // drive and start intake
                trajectory1.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
            )
            .andThen( // drive and stop intake and spin up shooter
                trajectory2.cmd()
                .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
            )
            .andThen(commandFactory.getDriveAndShootCommand(true, true)) // shoot
        );

        return routine;
    }

    public AutoRoutine repeatingCenterFuelLeft() {
        AutoRoutine routine = autoFactory.newRoutine("RepeatingCenterFuelLeft");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("RepeatingCenterFuelLeft1", 0);
        AutoTrajectory trajectory1 = routine.trajectory("RepeatingCenterFuelLeft1", 1);
        AutoTrajectory trajectory2 = routine.trajectory("RepeatingCenterFuelLeft1", 2);
        AutoTrajectory trajectory3 = routine.trajectory("RepeatingCenterFuelLeft1", 3);
        AutoTrajectory trajectory4 = routine.trajectory("RepeatingCenterFuelLeft1", 4);

        AutoTrajectory trajectory5 = routine.trajectory("RepeatingCenterFuelLeft2", 0);
        AutoTrajectory trajectory6 = routine.trajectory("RepeatingCenterFuelLeft2", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen( // the part that repeats
                // path 1
                trajectory1.cmd() // drive
                .andThen( // drive and start intake
                    trajectory2.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake, spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                // path 2
                .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
                .andThen(trajectory5.cmd()) // drive
                .andThen( // drive and start intake
                    trajectory6.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake and spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine repeatingCenterFuelRight() {
        AutoRoutine routine = autoFactory.newRoutine("RepeatingCenterFuelRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("RepeatingCenterFuelRight1", 0);
        AutoTrajectory trajectory1 = routine.trajectory("RepeatingCenterFuelRight1", 1);
        AutoTrajectory trajectory2 = routine.trajectory("RepeatingCenterFuelRight1", 2);
        AutoTrajectory trajectory3 = routine.trajectory("RepeatingCenterFuelRight1", 3);
        AutoTrajectory trajectory4 = routine.trajectory("RepeatingCenterFuelRight1", 4);

        AutoTrajectory trajectory5 = routine.trajectory("RepeatingCenterFuelRight2", 0);
        AutoTrajectory trajectory6 = routine.trajectory("RepeatingCenterFuelRight2", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen( // the part that repeats
                // path 1
                trajectory1.cmd() // drive
                .andThen( // drive and start intake
                    trajectory2.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake, spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                // path 2
                .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
                .andThen(trajectory5.cmd()) // drive
                .andThen( // drive and start intake
                    trajectory6.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake and spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine repeatingCenterFuelLeftCloser() {
        AutoRoutine routine = autoFactory.newRoutine("RepeatingCenterFuelLeftCloser");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("RepeatingCenterFuelLeft1Closer", 0);
        AutoTrajectory trajectory1 = routine.trajectory("RepeatingCenterFuelLeft1Closer", 1);
        AutoTrajectory trajectory2 = routine.trajectory("RepeatingCenterFuelLeft1Closer", 2);
        AutoTrajectory trajectory3 = routine.trajectory("RepeatingCenterFuelLeft1Closer", 3);
        AutoTrajectory trajectory4 = routine.trajectory("RepeatingCenterFuelLeft1Closer", 4);

        AutoTrajectory trajectory5 = routine.trajectory("RepeatingCenterFuelLeft2", 0);
        AutoTrajectory trajectory6 = routine.trajectory("RepeatingCenterFuelLeft2", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen( // the part that repeats
                // path 1
                trajectory1.cmd() // drive
                .andThen( // drive and start intake
                    trajectory2.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake, spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                // path 2
                .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
                .andThen(trajectory5.cmd()) // drive
                .andThen( // drive and start intake
                    trajectory6.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake and spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine repeatingCenterFuelRightCloser() {
        AutoRoutine routine = autoFactory.newRoutine("RepeatingCenterFuelRightCloser");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("RepeatingCenterFuelRight1Closer", 0);
        AutoTrajectory trajectory1 = routine.trajectory("RepeatingCenterFuelRight1Closer", 1);
        AutoTrajectory trajectory2 = routine.trajectory("RepeatingCenterFuelRight1Closer", 2);
        AutoTrajectory trajectory3 = routine.trajectory("RepeatingCenterFuelRight1Closer", 3);
        AutoTrajectory trajectory4 = routine.trajectory("RepeatingCenterFuelRight1Closer", 4);

        AutoTrajectory trajectory5 = routine.trajectory("RepeatingCenterFuelRight2", 0);
        AutoTrajectory trajectory6 = routine.trajectory("RepeatingCenterFuelRight2", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen( // the part that repeats
                // path 1
                trajectory1.cmd() // drive
                .andThen( // drive and start intake
                    trajectory2.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake, spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                // path 2
                .andThen(Commands.waitSeconds(1)) // wait for pivot to get down
                .andThen(trajectory5.cmd()) // drive
                .andThen( // drive and start intake
                    trajectory6.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(10), false))
                )
                .andThen( // drive and stop intake and spin up shooter
                    trajectory3.cmd()
                    .alongWith(intake.getSetIntakeVoltageCommand(Volts.of(0), false))
                    .alongWith(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(40)))
                )
                .andThen( // shoot for 4 seconds
                    commandFactory.getUpdateShootUtilCommand()
                    .alongWith(commandFactory.getShootCommand(
                        () -> ShootUtil.getShooterVelocity(), 
                        true, 
                        true, 
                        true
                    ))
                    .withTimeout(4)
                )
                .andThen(trajectory4.cmd()) // drive
                .repeatedly()
            )
        );

        return routine;
    }

    public AutoRoutine outpostFuel() {
        AutoRoutine routine = autoFactory.newRoutine("OutpostFuel");

        AutoTrajectory trajectory0 = routine.trajectory("OutpostFuel", 0);
        AutoTrajectory trajectory1 = routine.trajectory("OutpostFuel", 1);

        // routine composition
        routine.active().onTrue(
            trajectory0.resetOdometry()
            .andThen( // drive and bring pivot down
                trajectory0.cmd()
                .alongWith(
                    commandFactory.getSetPivotEncoderPositionUpCommand()
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(commandFactory.getSetPivotDownCommand())
                )
            )
            .andThen(Commands.waitSeconds(4)) // wait for outpost fuel
            .andThen(trajectory1.cmd()) // drive
            .andThen(commandFactory.getDriveAndShootCommand(true, true)) // shoot
        );

        return routine;
    }

    // ————— utils ————— // // ! is it utils or util

    public void drawSelectedAuto(String selectedCommandName) {
        Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory(nameFileMap.get(selectedCommandName));
        
        if (trajectory.isPresent()) {
            Constants.FieldConstants.FIELD2D.getObject("trajectory").setPoses(trajectory.get().getPoses());
        } else {
            Constants.FieldConstants.FIELD2D.getObject("trajectory").setPose(new Pose2d());
        }
    }
}