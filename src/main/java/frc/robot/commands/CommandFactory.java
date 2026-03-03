package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.FuelConstants;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;
import frc.robot.Constants;

public class CommandFactory {
    private final Controller controller;

    private final Drive drive;
    private final PoseEstimator poseEstimator;

    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    private final FuelSim fuelSimulation;

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
        return getShootCommand()
        .alongWith(getDriveWithJoysticksShooterCommand())
        .alongWith(
            (Constants.CURRENT_MODE == Constants.ROBOT_MODE.SIM ?
            getSimShootCommand() :
            new InstantCommand()).repeatedly()
        );
    }

    public Command getDriveAndIntakeAndShootCommand() {
        return getIntakeCommand()
        .alongWith(getDriveAndShootCommand());
    }

    // ! add an auto climb

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
                return new Rotation2d(Math.atan2( // negatives are to map xbox controller to the cartesian plane
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
            () -> ShootUtil.getRobotRotationToTarget()
        );
    }

    // ————— intake ————— //

    public Command getIntakeCommand() {
        return Commands.startEnd(
            () -> {
                intake.setIntakeVoltage(Volts.of(12));
                intake.setPivotPosition(FuelConstants.PIVOT_DOWN_ANGLE);
            },
            () -> intake.setIntakeVoltage(Volts.of(0))
        );
    }

    // ————— shoot ————— // 

    public Command getShootCommand() {
        return Commands.run(() -> shooter.runShooterState(ShootUtil.getShooterStateFromMapIterative( // start shooting
            poseEstimator.getPose(), 
            ShootUtil.getTargetPose(poseEstimator.getPose()),
            drive.getFieldRelativeSpeeds(), 
            3
        )))
        .alongWith( // allow shooting
            Commands.waitSeconds(1) // waits for shooter to get up to speed // ! does this work
            .andThen(
                hopper.getSetHopperVoltageCommand(Volts.of(5))
                .alongWith(shooter.getSetKickerVoltageCommand(Volts.of(5)))
            )
        ).finallyDo( // stop everything
            () -> {
                hopper.setHopperVoltage(Volts.of(0));
                shooter.runShooterState(new ShootUtil.ShooterState(RotationsPerSecond.of(0), Constants.HOOD_START_ANGLE));
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