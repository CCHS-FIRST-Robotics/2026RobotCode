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
    private final Voltage hopperVolts = Volts.of(10);
    private final Voltage kickerVolts = Volts.of(5); // ! 

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
        return getSlowDriveCommand(MetersPerSecond.of(2), MetersPerSecondPerSecond.of(10))
        .alongWith(getUpdateShootUtilCommand()).alongWith(getDriveWithJoysticksShooterCommand())
        .alongWith(getShootCommand(() -> ShootUtil.getShooterState()));
    }

    public Command getDriveAndIntakeAndShootCommand() {
        return getIntakeCommand()
        .alongWith(getDriveAndShootCommand());
    }

    public Command getSlowDriveCommand(LinearVelocity velocity, LinearAcceleration acceleration) {
        return Commands.startEnd(
            () -> {
                DriveConstants.MAX_ALLOWED_LINEAR_SPEED = velocity; // *
                DriveConstants.MAX_ALLOWED_ANGULAR_SPEED = RadiansPerSecond.of(DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond) / DriveConstants.TRACK_RADIUS);
                DriveConstants.MAX_ALLOWED_LINEAR_ACCEL = acceleration;
                DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL = RadiansPerSecondPerSecond.of(DriveConstants.MAX_ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / DriveConstants.TRACK_RADIUS);
            }, 
            () -> {
                DriveConstants.MAX_ALLOWED_LINEAR_SPEED = Constants.CURRENT_MODE == Constants.ROBOT_MODE.REAL ? MetersPerSecond.of(2) : MetersPerSecond.of(5); // *
                DriveConstants.MAX_ALLOWED_ANGULAR_SPEED = RadiansPerSecond.of(DriveConstants.MAX_ALLOWED_LINEAR_SPEED.in(MetersPerSecond) / DriveConstants.TRACK_RADIUS);
                DriveConstants.MAX_ALLOWED_LINEAR_ACCEL = MetersPerSecondPerSecond.of(20);
                DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL = RadiansPerSecondPerSecond.of(DriveConstants.MAX_ALLOWED_LINEAR_ACCEL.in(MetersPerSecondPerSecond) / DriveConstants.TRACK_RADIUS);
            }
        );
    }

    // ! add an auto climb

    public Command getUpdateShootUtilCommand() {
        return Commands.run(() -> ShootUtil.updateIterative(
            poseEstimator.getPose(), 
            ShootUtil.getTargetPose(poseEstimator.getPose()), 
            drive.getRobotRelativeSpeeds(), 3)
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
            () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped
            () -> controller.getLeftXWithDeadband(), 
            () -> controller.getRightXWithDeadband(),
            () -> {
                return new Rotation2d(Math.atan2( // negatives map xbox controller to the cartesian plane
                    -controller.getLeftXWithDeadband(), 
                    -controller.getLeftYWithDeadband()
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
                intake.setPivotPosition(FuelConstants.PIVOT_MAX_DOWN_ANGLE);
            },
            () -> intake.setIntakeVoltage(Volts.of(0))
        );
    }

    // ————— shoot ————— // 

    public Command getShootCommand(Supplier<ShooterState> shooterStateSupplier) {
        return Commands.run(() -> shooter.runShooterState(shooterStateSupplier.get()))
        .alongWith( // allow shooting
            Commands.waitSeconds(1) // waits for shooter to get up to speed
            .andThen(
                hopper.getSetHopperVoltageCommand(hopperVolts)
                .alongWith(shooter.getSetKickerVoltageCommand(kickerVolts))
            )
        ).alongWith(
            (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ?
            getSimShootCommand() :
            new InstantCommand()).repeatedly()
        ).finallyDo( // stop everything
            () -> {
                hopper.setHopperVoltage(Volts.of(0));
                // ! shooter.runShooterState(new ShootUtil.ShooterState(RotationsPerSecond.of(0), Constants.HOOD_START_ANGLE));
                shooter.setKickerVoltage(Volts.of(0));
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
}