package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.utils.ShooterCalculator;
import frc.robot.utils.ShooterCalculator.ShooterState;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;

public class Shoot extends Command {
    private final PoseEstimator poseEstimator;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;
    
    private final Pose2d targetPose;

    public Shoot(
        PoseEstimator poseEstimator,
        Intake intake,
        Hopper hopper,
        Shooter shooter,
        Pose2d targetPose
    ) {
        addRequirements(poseEstimator);
        addRequirements(intake);
        addRequirements(hopper);
        addRequirements(shooter);

        this.poseEstimator = poseEstimator;
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        
        this.targetPose = targetPose;
    }

    /**
     * one for shoot while holding button down !(make a new command)
     * - first, start the auto shoot command (periodic)
     * - start the hopper and kicker
     * - pivot the intake up // ! I need an easy way to change the speed that it pivots slot0configs is my love
     * - probably ANDTHEN reset the drive default command to have a heading supplier
     */

    @Override
    public void initialize() {
        hopper.setHopperVoltage(Volts.of(5));
        shooter.setKickerVoltage(Volts.of(5));
        // intake.setPivotPosition(FuelConstants.PIVOT_UP_ANGLE);
    }

    @Override
    public void execute() {
        shooter.runShooterState(ShooterCalculator.getShooterStateFromMap(poseEstimator.getPose(), targetPose));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.runShooterState(new ShooterState()); // ! idk if this actually means zero
    }
}