package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.subsystems.drive.*;

public class DriveWithPosition extends Command {
    private final Drive drive;
    private Pose2d targetPose;
    private Transform2d targetTransform;

    public DriveWithPosition(
        Drive drive,
        Pose2d targetPose
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
    }

    public DriveWithPosition(
        Drive drive,
        Transform2d targetTransform
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetTransform = targetTransform;
    }

    // ! add driving to a specific apriltag

    @Override
    public void initialize() {
        if(targetTransform != null){
            targetPose = drive.getPose().plus(targetTransform);
        }
    }

    @Override
    public void execute() {
        drive.runPosition(targetPose);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drive.getPose().getX() - targetPose.getX()) < 0.01
            && Math.abs(drive.getPose().getY() - targetPose.getY()) < 0.01
            && Math.abs(
                MathUtil.inputModulus(drive.getPose().getRotation().getRotations(), 0, 1)
                - MathUtil.inputModulus(targetPose.getRotation().getRotations(), 0, 1)
            ) < 0.005
            // ! the below would probably be good to add, more testing is necessary
            && drive.getChassisSpeeds().vxMetersPerSecond < 0.1
            && drive.getChassisSpeeds().vyMetersPerSecond < 0.1
            && drive.getChassisSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}