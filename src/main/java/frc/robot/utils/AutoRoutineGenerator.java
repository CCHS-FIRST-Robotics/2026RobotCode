package frc.robot.utils;

import choreo.auto.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.drive.*;

public class AutoRoutineGenerator {
    private final AutoFactory autoFactory;

    private final Drive drive;

    public AutoRoutineGenerator(
        Drive drive
    ) {
        autoFactory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::runAutoPosition, // ! hm, I think this needs a setposition function to exist
            DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false,
            drive
        );

        this.drive = drive;
    }

    // ————— testing routines ————— //

    public AutoRoutine test() {
        AutoRoutine routine = autoFactory.newRoutine("Test");

        // load trajectories
        AutoTrajectory trajectory = routine.trajectory("Test");

        // when routine begins, reset odometry, start trajectory
        routine.active().onTrue(
            trajectory.resetOdometry()
            .andThen(trajectory.cmd())
            .andThen(new DriveWithPosition(drive, trajectory.getFinalPose().get()))
        );

        return routine;
    }

    // ————— competition routines ————— //

    public Command backUp() {
        return new DriveWithPosition(drive, new Transform2d(-2, 0, new Rotation2d()));
    }
}