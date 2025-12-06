package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.*;

public class DriveWithPosition extends Command { // ! for some reason, rotation direction is unoptimized
    private final Drive drive;
    private final Pose2d targetPose;
    private final Twist2d twistSetpoint;

    // local constants // ! tune
    private final PIDController xPID = new PIDController(5, 0, 0);
    private final PIDController yPID = new PIDController(5, 0, 0);
    private final PIDController oPID = new PIDController(5, 0, 0);

    public DriveWithPosition(
        Drive drive,
        Pose2d targetPose
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
        this.twistSetpoint = new Twist2d();
    }

    public DriveWithPosition(
        Drive drive,
        Pose2d targetPose, 
        Twist2d twistSetpoint
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = targetPose;
        this.twistSetpoint = twistSetpoint;
    }

    public DriveWithPosition(
        Drive drive,
        Transform2d targetTransform
    ) {
        addRequirements(drive);
        this.drive = drive;
        this.targetPose = drive.getPose().plus(targetTransform);
        this.twistSetpoint = new Twist2d();
    }

    // ! add driving to a specific apriltag

    @Override
    public void execute() {
        // get position PID outputs
        double xOutput = xPID.calculate(drive.getPose().getX(), targetPose.getX());
        double yOutput = yPID.calculate(drive.getPose().getY(), targetPose.getY());
        double oOutput = oPID.calculate(drive.getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());

        // create chassisspeeds object with FOC
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds( // ! uhhh add twistsetpoints for choreo
            twistSetpoint.dx + xOutput,
            twistSetpoint.dy + yOutput,
            twistSetpoint.dtheta + oOutput,
            drive.getPose().getRotation()
        );

        drive.runVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        Pose2d robotPose = drive.getPose();
        return Math.abs(robotPose.getX() - targetPose.getX()) < 0.01
            && Math.abs(robotPose.getY() - targetPose.getY()) < 0.01
            && Math.abs(
                MathUtil.inputModulus(robotPose.getRotation().getRotations(), 0, 1)
                - MathUtil.inputModulus(targetPose.getRotation().getRotations(), 0, 1)
            ) < 0.005;
            // ! the below would probably be good to add, more testing is necessary
            // && drive.getSpeeds().vxMetersPerSecond < 0.1
            // && drive.getSpeeds().vyMetersPerSecond < 0.1
            // && drive.getSpeeds().omegaRadiansPerSecond < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}