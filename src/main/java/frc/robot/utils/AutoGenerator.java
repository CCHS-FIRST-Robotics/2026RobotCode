/**
 * Original code
 */

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.*;
import choreo.Choreo;
import choreo.auto.*;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import org.ironmaple.simulation.drivesims.*;
import org.littletonrobotics.junction.Logger;
import java.util.*;
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

    // tells the auto drawer what file to get the trajectory from. this was necessary for the autos that have a different name from the main file they pull from
    // (e.g. RepeatingCenterFuelLeft follows both RepeatingCenterFuelLeft1 and RepeatingCenterFuelLeft2 in code, but the main path is in RepeatingCenterFuelLeft1, so we pull the trajectory to draw from there)
    private final HashMap<String, String> nameFileMap = new HashMap<String, String>();

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
                if (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM && simTeleportOnOdometryReset) {
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

    // sim only: resetting odometry also teleports the sim robot to that pose (so sim autos start in the right place).
    // the smart start test turns this off, so the old reset-odometry behavior can't cheat by moving the robot onto the path
    public static boolean simTeleportOnOdometryReset = true;

    // ————— smart auto start ————— //

    private static final double MAX_DRIVE_TO_START = 1.5; // m; farther than this and vision is probably wrong (or the wrong auto is selected), so don't trust it
    private static final double DRIVE_TO_START_TIMEOUT = 2.0; // s

    public enum StartMethod {
        RESET,          // tell the robot it's exactly at the path start (the old behavior)
        TRUST_VISION,   // keep the vision pose and start the path; the follower's PID pulls the robot onto the path
        DRIVE_TO_START  // drive to the path start, then start the path (costs time)
    }

    // what to do when vision has a fix but the robot is farther off than AutoStartHelper's "close" limits (the LEDs are red)
    public static StartMethod FAR_START_METHOD = StartMethod.DRIVE_TO_START;

    // sim test hook: forces one method regardless of the situation (null = decide normally)
    public static StartMethod forcedStartMethod = null;

    /**
     * Runs at the very start of an auto, in place of trajectory.resetOdometry(). Decides based on what vision says:
     *   - vision has a fix and the robot is close to the path start (within AutoStartHelper's "close" limits): TRUST_VISION
     *   - vision has a fix and the robot is farther off (up to MAX_DRIVE_TO_START): FAR_START_METHOD
     *   - no vision, way too far off, or ENABLE_SMART_AUTO_START is off: RESET (the old behavior)
     */
    private Command smartStart(AutoTrajectory firstTrajectory) {
        return Commands.defer(() -> {
            Optional<Pose2d> start = firstTrajectory.getInitialPose();
            if (start.isEmpty()) {
                return logStartDecision(StartMethod.RESET, "path has no start pose", firstTrajectory, start);
            }

            Transform2d error = new Transform2d(poseEstimator.getPose(), start.get());
            double distance = error.getTranslation().getNorm();
            double angle = Math.abs(error.getRotation().getRadians());
            String errorText = String.format("%.0f cm, %.0f deg off", distance * 100, Math.toDegrees(angle));

            if (forcedStartMethod != null) {
                return logStartDecision(forcedStartMethod, "forced by test, " + errorText, firstTrajectory, start);
            }
            if (!Constants.ENABLE_SMART_AUTO_START) {
                return logStartDecision(StartMethod.RESET, "smart start is off", firstTrajectory, start);
            }
            if (poseEstimator.getSecondsSinceVision() > AutoStartHelper.MAX_SECONDS_SINCE_VISION) {
                return logStartDecision(StartMethod.RESET, "no recent vision", firstTrajectory, start);
            }
            if (distance <= AutoStartHelper.CLOSE_TRANSLATION && angle <= AutoStartHelper.CLOSE_ROTATION) {
                return logStartDecision(StartMethod.TRUST_VISION, errorText, firstTrajectory, start);
            }
            if (distance <= MAX_DRIVE_TO_START) {
                return logStartDecision(FAR_START_METHOD, errorText, firstTrajectory, start);
            }
            return logStartDecision(StartMethod.RESET, errorText + ", too far to trust", firstTrajectory, start);
        }, Set.of(drive, poseEstimator));
    }

    private int startDecisionCount = 0;

    private Command logStartDecision(StartMethod method, String reason, AutoTrajectory firstTrajectory, Optional<Pose2d> start) {
        String decision = method + " (" + reason + ")";
        System.out.println("[AutoGenerator] auto start: " + decision);
        Logger.recordOutput("outputs/autoStart/decision", decision);
        Logger.recordOutput("outputs/autoStart/decisionCount", ++startDecisionCount); // logs only store changes, so this marks repeats of the same decision
        return switch (method) {
            case RESET -> firstTrajectory.resetOdometry();
            case TRUST_VISION -> Commands.none();
            case DRIVE_TO_START -> new DriveWithPosition(drive, poseEstimator, start.get(), false).withTimeout(DRIVE_TO_START_TIMEOUT);
        };
    }


    // ————— testing routines ————— //

    public AutoRoutine test() { // to use this just draw a sufficiently complex trajectory in choreo to test if the robot can follow it well
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("Test", 0);

        // routine composition
        routine.active().onTrue(
            smartStart(trajectory0)
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
            smartStart(trajectory0)
            .andThen(trajectory0.cmd()) // drive to neutral zone
            .andThen( // continually drive in a circle
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
            smartStart(trajectory0)
            .andThen(trajectory0.cmd()) // drive to neutral zone
            .andThen( // continually drive in a circle
                trajectory1.cmd()
                .repeatedly()
            )
        );

        return routine;
    }

    /**
     * goes through the left trench, intakes along a path, and then returns to the hub and shoots
     */
    public AutoRoutine centerFuelLeft() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelLeft");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelLeft", 0);
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelLeft", 1);
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelLeft", 2);

        // routine composition
        routine.active().onTrue(
            smartStart(trajectory0)
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

    /**
     * goes through the right trench, intakes along a path, and then returns to the hub and shoots
     */
    public AutoRoutine centerFuelRight() {
        AutoRoutine routine = autoFactory.newRoutine("CenterFuelRight");

        // load trajectories
        AutoTrajectory trajectory0 = routine.trajectory("CenterFuelRight", 0);
        AutoTrajectory trajectory1 = routine.trajectory("CenterFuelRight", 1);
        AutoTrajectory trajectory2 = routine.trajectory("CenterFuelRight", 2);

        // routine composition
        routine.active().onTrue(
            smartStart(trajectory0)
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

    /**
     * loops centerFuelLeft but intakes along two different paths
     */
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
            smartStart(trajectory0)
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

    /**
     * loops centerFuelRight but intakes along two different paths
     */
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
            smartStart(trajectory0)
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

    /**
     * same as repeatingCenterFuelLeft but it shoots closer to the hub
     */
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
            smartStart(trajectory0)
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

    /**
     * same as repeatingCenterFuelRight but it shoots closer to the hub
     */
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
            smartStart(trajectory0)
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

    /**
     * drives from the center to the outpost, then shoots
     */
    public AutoRoutine outpostFuel() {
        AutoRoutine routine = autoFactory.newRoutine("OutpostFuel");

        AutoTrajectory trajectory0 = routine.trajectory("OutpostFuel", 0);
        AutoTrajectory trajectory1 = routine.trajectory("OutpostFuel", 1);

        // routine composition
        routine.active().onTrue(
            smartStart(trajectory0)
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

    // ————— utils ————— //

    // what's currently drawn, so the trajectory file is only loaded when the selection or alliance changes
    // (this runs every loop while disabled, and loading parses the file from disk)
    private String drawnAutoName = null;
    private Alliance drawnAlliance = null;
    private Optional<Pose2d> selectedAutoStartPose = Optional.empty();

    public void drawSelectedAuto(String selectedCommandName) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (selectedCommandName.equals(drawnAutoName) && alliance == drawnAlliance) {
            return;
        }
        drawnAutoName = selectedCommandName;
        drawnAlliance = alliance;

        // autos that aren't a choreo path (e.g. BackUpAndShoot, "do nothing") have nothing to draw
        String fileName = nameFileMap.get(selectedCommandName);
        Optional<Trajectory<SwerveSample>> trajectory = fileName == null ? Optional.empty() : Choreo.loadTrajectory(fileName);

        if (trajectory.isPresent() && trajectory.get().getPoses().length > 0) {
            Pose2d[] poses = Arrays.stream(trajectory.get().getPoses())
                .map( // flip poses based on alliance
                    pose -> alliance == Alliance.Blue ?
                    pose :
                    Constants.FieldConstants.calculateAllianceFlippedPose(pose)
                )
                .toArray(Pose2d[]::new);
            Constants.FieldConstants.FIELD2D.getObject("trajectory").setPoses(poses); // create the trajectory on the field2d
            selectedAutoStartPose = Optional.of(poses[0]);
        } else {
            Constants.FieldConstants.FIELD2D.getObject("trajectory").setPoses();
            selectedAutoStartPose = Optional.empty();
        }
    }

    // where the robot needs to be placed for the selected auto (empty if the auto doesn't start from a fixed spot)
    public Optional<Pose2d> getSelectedAutoStartPose() {
        return selectedAutoStartPose;
    }
}