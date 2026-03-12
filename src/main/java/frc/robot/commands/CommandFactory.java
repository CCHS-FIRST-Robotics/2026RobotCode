package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
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
    private final AngularVelocity hopperVelocity = RotationsPerSecond.of(15);
    private final AngularVelocity kickerVelocity = RotationsPerSecond.of(30);

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
        return getIntakeCommand()
        .alongWith(getDriveWithJoysticksIntakeCommand());
    }

    public Command getDriveAndShootCommand() {
        return getSlowDriveCommand(
            MetersPerSecond.of(1.5), 
            RotationsPerSecond.of(0.25), 
            MetersPerSecondPerSecond.of(10),
            RotationsPerSecondPerSecond.of(10 / DriveConstants.TRACK_RADIUS)
        )
        .alongWith(getUpdateShootUtilCommand()).alongWith(getDriveWithJoysticksShooterCommand())
        .alongWith(getShootCommand(() -> ShootUtil.getShooterState()));
    }

    public Command getDriveAndIntakeAndShootCommand() {
        return getIntakeCommand()
        .alongWith(getDriveAndShootCommand());
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

    // ————— drive ————— //

    public Command getDriveWithJoysticksCommand() {
        return new DriveWithJoysticks(
            drive,
            poseEstimator,
            () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped, x velocity relative to field is "forward" from driver perspective
            () -> controller.getLeftXWithDeadband(), 
            () -> controller.getRightXWithDeadband(),
            null
        );
    }

    public Command getDriveWithJoysticksIntakeCommand() {
        return new DriveWithJoysticks(
            drive, 
            poseEstimator, 
            () -> -controller.getLeftYWithDeadband(0.8), // xbox controller is flipped
            () -> controller.getLeftXWithDeadband(0.8), 
            () -> controller.getRightXWithDeadband(0.8),
            () -> {
                return new Rotation2d(Math.atan2( // negatives map xbox controller to the cartesian plane
                    -controller.getLeftXWithDeadband(0.8), 
                    -controller.getLeftYWithDeadband(0.8)
                ) + Math.PI); // intake is on back of robot
            }
        );
    }

    public Command getDriveWithJoysticksShooterCommand() {
        return new DriveWithJoysticks(
            drive, 
            poseEstimator, 
            () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped
            () -> controller.getLeftXWithDeadband(), 
            () -> controller.getRightXWithDeadband(),
            () -> ShootUtil.getRobotRotation()
        );
    }

    // ————— intake ————— //

    public Command getIntakeCommand() {
        return Commands.startEnd(
            () -> {
                intake.setIntakeVoltage(intakeVolts);
                // intake.setPivotPosition(FuelConstants.PIVOT_MAX_DOWN_ANGLE); // ! do this at the end of shoot command
            },
            () -> intake.setIntakeVoltage(Volts.of(0))
        );
    }

    // ————— shoot ————— // 

    public Command getShootCommand(Supplier<ShooterState> shooterStateSupplier) {
        return Commands.run(() -> shooter.runShooterState(shooterStateSupplier.get()))
        .alongWith( // allow shooting
            Commands.waitSeconds(1) // waits for shooter to get up to speed // ! waitUntil
            .andThen(
                // intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_UP_ANGLE)
                hopper.getSetHopperVelocityCommand(hopperVelocity)
                .alongWith(shooter.getSetKickerVelocityCommand(kickerVelocity))
            )
        ).alongWith(
            (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ?
            getSimShootCommand() :
            new InstantCommand()).repeatedly()
        ).finallyDo( // stop everything
            () -> {
                // intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE)
                hopper.setHopperVelocity(RotationsPerSecond.of(0));
                // shooter.runShooterState(new ShootUtil.ShooterState(RotationsPerSecond.of(0), Constants.HOOD_START_ANGLE));
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
        ).andThen(Commands.waitSeconds(0.5));
    }

    // ————— util ————— //

    public Command getUpdateShootUtilCommand() {
        return Commands.run(() -> ShootUtil.updateIterative(
            poseEstimator.getPose(), 
            ShootUtil.getTargetPose(poseEstimator.getPose()), 
            drive.getRobotRelativeSpeeds(), 3)
        );
    }
}