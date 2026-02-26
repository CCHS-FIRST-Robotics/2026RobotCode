package frc.robot.commands;

import static edu.wpi.first.units.Units.*;


import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.geometry.*;
import java.util.function.*;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.poseEstimator.*;
import frc.robot.subsystems.fuelIO.*;
import frc.robot.subsystems.fuelIO.intake.*;
import frc.robot.subsystems.fuelIO.hopper.*;
import frc.robot.subsystems.fuelIO.shooter.*;
import frc.robot.utils.*;
import frc.robot.utils.Calculator.*;
import frc.robot.Constants;

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
        Pose2d robotPose = poseEstimator.getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        Translation2d robotToTarget = robotPose.getTranslation().minus(targetPose.getTranslation());
        shooter.runShooterState(Calculator.getShooterStateFromMap(robotToTarget.getNorm()));
        
        Logger.recordOutput("outputs/commands/shootCommand/target", targetPose);
        Logger.recordOutput("outputs/commands/shootCommand/distFromTarget", robotToTarget.getNorm());
    }

    @Override
    public void end(boolean interrupted) {
        hopper.setHopperVoltage(Volts.of(0));
        shooter.runShooterState(new ShooterState(RotationsPerSecond.of(0), Constants.HOOD_START_ANGLE));
        shooter.setKickerVoltage(Volts.of(0));
    }
}