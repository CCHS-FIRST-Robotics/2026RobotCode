package frc.robot.commands;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import java.util.function.*;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;
import frc.robot.utils.ShooterCalculator.*;

public class ShootCommand extends Command {
    private final PoseEstimator poseEstimator;
    
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;
    
    private final Supplier<Pose2d> targetPoseSupplier;

    public ShootCommand(
        PoseEstimator poseEstimator,
        Intake intake,
        Hopper hopper,
        Shooter shooter,
        Supplier<Pose2d> targetPoseSupplier
    ) {
        this.poseEstimator = poseEstimator;
        
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;
        
        this.targetPoseSupplier = targetPoseSupplier;
    }

    @Override
    public void initialize() {
        intake.setPivotPosition(FuelConstants.PIVOT_UP_ANGLE);
        hopper.setHopperVoltage(Volts.of(5));
        shooter.setKickerVoltage(Volts.of(5));
    }

    @Override
    public void execute() {
        // shooter.runShooterState(ShooterCalculator.getShooterStateFromMap(poseEstimator.getPose(), targetPoseSupplier.get()));
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setHopperVoltage(Volts.of(0));
        // shooter.runShooterState(new ShooterState(RotationsPerSecond.of(0), FuelConstants.HOOD_DOWN_ANGLE));
        shooter.setKickerVoltage(Volts.of(0));
    }
}