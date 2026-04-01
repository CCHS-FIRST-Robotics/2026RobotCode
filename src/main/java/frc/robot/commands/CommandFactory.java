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
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class CommandFactory {
    private final Controller controller;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Shooter shooter;

    private final FuelSim fuelSimulation;

    private final Voltage intakeVolts = Volts.of(10);
    private final AngularVelocity kickerVelocity = RotationsPerSecond.of(60);

    private boolean pivotUp = Constants.PIVOT_START_ANGLE == FuelConstants.PIVOT_MAX_UP_ANGLE;
    
    public CommandFactory(
        Controller controller,
        Drive drive,
        PoseEstimator poseEstimator,
        Intake intake,
        Shooter shooter,
        FuelSim fuelSimulation
    ) {
        this.controller = controller;

        this.drive = drive;
        this.poseEstimator = poseEstimator;
        
        this.intake = intake;
        this.shooter = shooter;

        this.fuelSimulation = fuelSimulation;
    }

    // ————— processed ————— //

    public Command getDriveAndIntakeCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

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
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return getSlowDriveCommand(
            MetersPerSecond.of(1), 
            DriveConstants.MAX_ALLOWED_ANGULAR_SPEED, 
            DriveConstants.MAX_ALLOWED_LINEAR_ACCEL, 
            DriveConstants.MAX_ALLOWED_ANGULAR_ACCEL
        )
        .alongWith(getUpdateShootUtilCommand())
        .alongWith(getDriveWithJoysticksShooterCommand())
        .alongWith(getShootCommand(() -> ShootUtil.getShooterVelocity(), usePivot));
    }

    public Command getDriveAndIntakeAndShootCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return getIntakeCommand()
        .alongWith(getDriveAndShootCommand(false));
    }

    public Command getCheckMotorsCommand() {
        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return getSetPivotDownCommand()
        .andThen(Commands.waitUntil(() -> intake.pivotDown()))
        .andThen(intake.getSetIntakeVoltageCommand(intakeVolts))
        .andThen(Commands.waitSeconds(1))
        .andThen(intake.getSetIntakeVoltageCommand(Volts.of(0)))
        .andThen(getSetPivotUpCommand())
        .andThen(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(30)))
        .andThen(shooter.getSetKickerVelocityCommand(RotationsPerSecond.of(60)))
        .andThen(Commands.waitSeconds(2))
        .andThen(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(0)))
        .andThen(shooter.getSetKickerVelocityCommand(RotationsPerSecond.of(0)))
        .andThen(Commands.waitSeconds(2))
        .andThen(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(-5)))
        .andThen(shooter.getSetKickerVelocityCommand(RotationsPerSecond.of(5)))
        .andThen(Commands.waitSeconds(120))
        .andThen(shooter.getSetShooterVelocityCommand(RotationsPerSecond.of(0)))
        .andThen(shooter.getSetKickerVelocityCommand(RotationsPerSecond.of(0)));
    }

    // ————— drive ————— //

    public Command getDriveWithJoysticksCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return new InstantCommand();
        }

        return new DriveWithJoysticks(
            drive,
            poseEstimator,
            () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped, x velocity relative to field is "forward" from driver perspective
            () -> controller.getLeftXWithDeadband(), 
            () -> controller.getRightXWithDeadband(),
            null, 
            true,
            false
        );
    }

    public Command getDriveWithJoysticksIntakeCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return new DriveWithJoysticks(
            drive, 
            poseEstimator, 
            () -> -controller.getLeftYWithDeadband(0.6), // xbox controller is flipped
            () -> controller.getLeftXWithDeadband(0.6), 
            () -> controller.getRightXWithDeadband(0.6),
            () -> { // turn the intake in the direction of the left joystick
                double x = controller.getLeftXWithDeadband(0.6);
                double y = controller.getLeftYWithDeadband(0.6);

                if (x == 0 && y == 0) {
                    return null;
                }
                
                return new Rotation2d(
                    Math.atan2(-x, -y) // negatives map xbox controller to the cartesian plane
                    - (
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ?
                        0 : 
                        Math.PI
                    ) // flip for alliance color
                );
            }, 
            true,
            false
        );
    }

    public Command getDriveWithJoysticksShooterCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return Commands.either(
            new DriveWithJoysticks(
                drive, 
                poseEstimator, 
                () -> -controller.getLeftYWithDeadband(), // xbox controller is flipped
                () -> controller.getLeftXWithDeadband(), 
                () -> controller.getRightXWithDeadband(),
                () -> ShootUtil.getRobotRotation(),
                false,
                true
            ), 
            new DriveWithJoysticks(
                drive, 
                poseEstimator, 
                () -> 0, // xbox controller is flipped
                () -> 0, 
                () -> 0,
                () -> ShootUtil.getRobotRotation(),
                false,
                true
            ), 
            () -> Constants.ENABLE_SHOOT_ON_THE_MOVE
        );
    }

    public Command getSlowDriveCommand(
        LinearVelocity linearVelocity, 
        AngularVelocity angularVelocity, 
        LinearAcceleration linearAcceleration,
        AngularAcceleration angularAcceleration
    ) {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR) {
            return new InstantCommand();
        }

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
        if (!Constants.INSTANTIATE_INTAKE) {
            return new InstantCommand();
        }
        
        return Commands.startEnd(
            () -> intake.setIntakeVoltage(intakeVolts),
            () -> intake.setIntakeVoltage(Volts.of(0))
        );
    }

    public Command getSetPivotUpCommand() {
        if (!Constants.INSTANTIATE_INTAKE) {
            return new InstantCommand();
        }

        return intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_UP_ANGLE)
        .andThen(Commands.runOnce(() -> {pivotUp = true;}));
    }

    public Command getSetPivotDownCommand() {
        if (!Constants.INSTANTIATE_INTAKE) {
            return new InstantCommand();
        }

        return intake.getSetPivotPositionCommand(FuelConstants.PIVOT_MAX_DOWN_ANGLE)
        .andThen(Commands.runOnce(() -> {pivotUp = false;}));
    }

    public Command getTogglePivotCommand() {
        if (!Constants.INSTANTIATE_INTAKE) {
            return new InstantCommand();
        }

        return Commands.either(
            getSetPivotDownCommand(),
            getSetPivotUpCommand(),
            () -> pivotUp
        );
    }

    // ————— shoot ————— // 

    public Command getShootCommand(Supplier<AngularVelocity> shooterVelocitySupplier, boolean usePivot) {
        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return Commands.run(() -> shooter.setShooterVelocity(shooterVelocitySupplier.get()))
        .alongWith( // allow shooting
            Commands.waitSeconds(0.1)
            .andThen(Commands.waitUntil(() -> drive.atThetaSetpoint())) // waits for drive rotation to be correct
            .andThen(Commands.waitUntil(() -> shooter.getShooterUpToSpeed())) // waits for shooter to get up to speed
            .andThen(shooter.getSetKickerVelocityCommand(RotationsPerSecond.of(-10))) // run backwards to avoid the balls that are already lodged in there
            .andThen(Commands.waitSeconds(0.25))
            .andThen(shooter.getSetKickerVelocityCommand(kickerVelocity))
        )
        .alongWith( // move pivot back and forth
            usePivot ? 
            Commands.either(
                (
                    Commands.waitSeconds(0.25)
                    .andThen(getSetPivotUpCommand())
                    .andThen(Commands.waitSeconds(0.5))
                    .andThen(getSetPivotDownCommand())
                ).repeatedly(), 
                Commands.waitSeconds(0.5)
                .andThen(getSetPivotUpCommand()), 
                () -> Constants.ENABLE_PIVOT_AGITATION
            ) : 
            new InstantCommand()
        )
        .alongWith(
            Commands.waitSeconds(0.1)
            .andThen(Commands.waitUntil(() -> shooter.getShooterUpToSpeed()))
            .andThen(
                Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ?
                getSimShootCommand() :
                new InstantCommand()
            )
        )
        .finallyDo( // stop everything
            () -> {
                if (usePivot) {
                    intake.setPivotPosition(FuelConstants.PIVOT_MAX_DOWN_ANGLE);
                    pivotUp = false;
                }
                if (Constants.CURRENT_BUTTON_BINDINGS != Constants.BUTTON_BINDINGS.TESTING_SHOOTER_MAP) {
                    shooter.setShooterVelocity(RotationsPerSecond.of(0));
                }
                shooter.setKickerVelocity(RotationsPerSecond.of(0));
            }
        );
    }

    public Command getSimShootCommand() {
        if (!Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return Commands.runOnce(
            () -> {
                if (Constants.REALISTIC_SIM) {
                    if (intake.getHopperEmpty()) {
                        return;
                    }
                    intake.subtractHopperFuel();
                }

                fuelSimulation.launchFuel(
                    () -> shooter.getShooterLinearVelocity(), 
                    () -> Degrees.of(90).minus(FuelConstants.HOOD_ANGLE), // shot angle
                    Rotations.of(0),
                    FuelConstants.SHOOTER_POSITION.plus(new Transform3d(0, Inches.of(-3).in(Meters), 0, new Rotation3d()))
                );

                fuelSimulation.launchFuel(
                    () -> shooter.getShooterLinearVelocity(), 
                    () -> Degrees.of(90).minus(FuelConstants.HOOD_ANGLE), // shot angle
                    Rotations.of(0),
                    FuelConstants.SHOOTER_POSITION.plus(new Transform3d(0, Inches.of(3).in(Meters), 0, new Rotation3d()))
                );
            }
        )
        .andThen(
            Commands.either(
                Commands.waitSeconds(0.1),
                Commands.waitSeconds(0.5),
                () -> Constants.REALISTIC_SIM
            )
        ) // time between shots
        .repeatedly();
    }

    // ————— util ————— //

    public Command getUpdateShootUtilCommand() {
        if (!Constants.INSTANTIATE_DRIVE_AND_POSEESTIMATOR
            || !Constants.INSTANTIATE_INTAKE
            || !Constants.INSTANTIATE_SHOOTER
        ) {
            return new InstantCommand();
        }

        return Commands.either(
            Commands.run(
                () -> ShootUtil.updateIterative(
                    poseEstimator.getPose(), 
                    ShootUtil.getTargetPose(poseEstimator.getPose()), 
                    drive.getFieldRelativeSpeeds(), 3
                )
            ), 
            Commands.run(
                () -> ShootUtil.update(
                    poseEstimator.getPose(), 
                    ShootUtil.getTargetPose(poseEstimator.getPose())
                )
            ), 
            () -> Constants.ENABLE_SHOOT_ON_THE_MOVE
        );
    }
}