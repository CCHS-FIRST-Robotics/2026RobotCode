package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import java.util.function.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.FuelConstants;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;
import frc.robot.utils.ShootUtil.ShooterState;
import frc.robot.Constants;

public class CommandFactory {
    private final Controller controller;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    private final FuelSim fuelSimulation;

    private final Voltage intakeVolts = Volts.of(10);
    private final AngularVelocity hopperVelocity = RotationsPerSecond.of(10);
    private final AngularVelocity kickerVelocity = RotationsPerSecond.of(60);

    public CommandFactory(
        Controller controller,
        Drive drive,
        PoseEstimator poseEstimator,
        Intake intake,
        Hopper hopper,
        Shooter shooter,
        FuelSim fuelSimulation
    ) {
        this.controller = controller;

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;

        this.fuelSimulation = fuelSimulation;
    }

    // ————— processed ————— //

    public Command getDriveAndIntakeCommand() {
        return getSlowDriveCommand(
            MetersPerSecond.of(1), 
            DriveConstants.MAX_ALLOWED_ANGULAR_SPEED, 
            DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
            DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
        )
        .alongWith(getDriveWithJoysticksIntakeCommand())
        .alongWith(getIntakeCommand());
    }

    public Command getDriveAndShootCommand(boolean usePivot) {
        return getSlowDriveCommand(
            MetersPerSecond.of(1), 
            DriveConstants.MAX_ALLOWED_ANGULAR_SPEED, 
            DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
            DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
        )
        .alongWith(getUpdateShootUtilCommand())
        .alongWith(getDriveWithJoysticksShooterCommand())
        .alongWith(getShootCommand(() -> ShootUtil.getShooterState(), () -> usePivot));
    }

    public Command getDriveAndIntakeAndShootCommand() {
        return getIntakeCommand()
        .alongWith(getDriveAndShootCommand(false));
    }

    // ————— drive ————— //

    public Command getDriveWithJoysticksCommand() {
        return new DriveWithJoysticks(
            drive,
            poseEstimator,
            () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped, x velocity relative to field is "forward" from driver perspective
            () -> controller.getLeftXWithDeadband(), 
            () -> controller.getRightXWithDeadband(),
            null, 
            false, 
            () -> Constants.TRENCH_ALIGN
        );
    }

    public Command getDriveWithJoysticksIntakeCommand() {
        return new DriveWithJoysticks(
            drive, 
            poseEstimator, 
            () -> -controller.getLeftYWithDeadband(0.6), // xbox controller is flipped
            () -> controller.getLeftXWithDeadband(0.6), 
            () -> controller.getRightXWithDeadband(0.6),
            () -> {
                double x = controller.getLeftXWithDeadband(0.6);
                double y = controller.getLeftYWithDeadband(0.6);

                if (x == 0 && y == 0) {
                    return null;
                }
                
                return new Rotation2d(
                    Math.atan2(-x, -y) // negatives map xbox controller to the cartesian plane
                    + Math.PI // intake is on back of robot
                    - (
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ?
                        0 : 
                        Math.PI
                    ) // flip for alliance color
                );
            }, 
            false, 
            () -> Constants.TRENCH_ALIGN
        );
    }

    public Command getDriveWithJoysticksShooterCommand() {
        return new DriveWithJoysticks(
            drive, 
            poseEstimator, 
            () -> 0, // xbox controller is flipped
            () -> 0, 
            () -> 0,
            () -> ShootUtil.getRobotRotation(),
            true, 
            () -> Constants.TRENCH_ALIGN
        );
    }

    public Command getSlowDriveCommand(
        LinearVelocity linearVelocity, 
        AngularVelocity angularVelocity, 
        LinearAcceleration linearAcceleration,
        AngularAcceleration angularAcceleration
    ) {
        return Commands.startEnd(
            () -> {
                DriveConstants.ALLOWED_LINEAR_SPEED = linearVelocity;
                DriveConstants.ALLOWED_ANGULAR_SPEED = angularVelocity;
                DriveConstants.ALLOWED_LINEAR_ACCEL = linearAcceleration;
                DriveConstants.ALLOWED_ANGULAR_ACCEL = angularAcceleration;
            }, 
            () -> {
                DriveConstants.ALLOWED_LINEAR_SPEED = DriveConstants.MAX_ALLOWED_LINEAR_SPEED;
                DriveConstants.ALLOWED_ANGULAR_SPEED = DriveConstants.MAX_ALLOWED_ANGULAR_SPEED;
                DriveConstants.ALLOWED_LINEAR_ACCEL = DriveConstants.MAX_ALLOWED_LINEAR_ACCEL;
                DriveConstants.ALLOWED_ANGULAR_ACCEL = DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL;
            }
        );
    }

    // ————— intake ————— //

    public Command getIntakeCommand() {
        return Commands.startEnd(
            () -> intake.setIntakeVoltage(intakeVolts),
            () -> intake.setIntakeVoltage(Volts.of(0))
        );
    }

    // ————— shoot ————— // 

    public Command getShootCommand(Supplier<ShooterState> shooterStateSupplier, BooleanSupplier usePivotSupplier) {
        return Commands.run(() -> shooter.runShooterState(shooterStateSupplier.get()))
        .alongWith( // allow shooting
            Commands.waitSeconds(0.1)
            .andThen(Commands.waitUntil(() -> shooter.getShooterUpToSpeed())) // waits for shooter to get up to speed
            .andThen(
                (
                    usePivotSupplier.getAsBoolean() ? 
                    Commands.waitSeconds(1)
                    .andThen(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_UP_ANGLE)) : 
                    Commands.waitSeconds(1)
                    .andThen(intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE))
                )
                .alongWith(
                    Commands.waitSeconds(0.5)
                    .andThen(hopper.getSetHopperVelocityCommand(hopperVelocity))
                )
                .alongWith(shooter.getSetKickerVelocityCommand(kickerVelocity))
            )
        )
        .alongWith(
            Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ?
            getSimShootCommand().repeatedly() :
            new InstantCommand()
        )
        .finallyDo( // stop everything
            () -> {
                if (usePivotSupplier.getAsBoolean()) {
                    intake.setPivotPosition(FuelConstants.PIVOT_MAX_DOWN_ANGLE);
                }
                hopper.setHopperVelocity(RotationsPerSecond.of(0));
                shooter.runShooterState(new ShootUtil.ShooterState(RotationsPerSecond.of(0), Constants.HOOD_START_ANGLE));
                shooter.setKickerVelocity(RotationsPerSecond.of(0));
            }
        );
    }

    public Command getSimShootCommand() {
        return Commands.runOnce(
            () -> {
                if (Constants.REALISTIC_SIM) {
                    if (hopper.getHopperEmpty()) {
                        return;
                    }
                    hopper.shootFuel();
                }

                fuelSimulation.launchFuel(
                    () -> shooter.getShooterLinearVelocity(), 
                    () -> shooter.getHoodShotAngle(),
                    Rotations.of(0),
                    FuelConstants.SHOOTER_POSITION
                );
            }
        )
        .andThen(Commands.waitSeconds(0.5));
    }

    // ————— util ————— //

    public Command getUpdateShootUtilCommand() {
        return Commands.run(
            () -> ShootUtil.updateIterative(
                poseEstimator.getPose(), 
                ShootUtil.getTargetPose(poseEstimator.getPose()), 
                drive.getFieldRelativeSpeeds(), 3
            )
        );
    }
}