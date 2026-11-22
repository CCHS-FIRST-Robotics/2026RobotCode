/**
 * Original code
 */

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.Logger;
import java.util.*;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.TagObservation;
import frc.robot.subsystems.poseEstimator.vision.VisionConstants;

/**
 * Finds each camera's robot-to-camera transform, so you don't have to measure camera positions and angles by hand.
 *
 * HOW TO USE:
 *   1. Put the robot somewhere you know relative to one AprilTag (e.g. centered in front of it, 2 m back, facing it).
 *      That's the only measurement needed: the robot's position relative to the tag, plus the tag's height.
 *   2. Run this command and keep the robot still (or rotate it very slowly to give the camera more views).
 *   3. Every camera that can see the tag gets a result, printed to the console as a line you can paste into
 *      VisionConstants.robotToCameraTransforms, and logged under outputs/cameraCalibration/.
 *   4. Cameras pointing different directions can't all see one tag, so re-place the robot and run it again for those.
 *
 * HOW IT WORKS: the camera measures camera->tag. You give tag->robot. So for every frame,
 *   robot->camera = (tag->robot)^-1 + (camera->tag)^-1
 * The result is averaged over all good frames, with outliers removed. If the robot moves during the run, the
 * odometry-only pose estimate (not vision, since vision uses the camera transforms being solved for) tracks how far.
 *
 * TAG FRAME (WPILib / PhotonVision convention): origin at the center of the tag, +X points straight out of the
 * tag's face, +Y points to the RIGHT as you stand looking at the tag, +Z points up. The robot's origin is the center
 * of its frame on the floor, so tag->robot's Z is minus the tag's center height.
 */
public class CalibrateCameras extends Command {
    // ————— tuning ————— //

    private static final double SETTLE_TIME = 0.25; // s ignored at the start (also drops frames from before the robot was placed)
    private static final double MAX_AMBIGUITY = 0.15; // single-tag pose ambiguity; higher means the tag's orientation might be flipped
    private static final double MAX_TAG_DISTANCE = 5.0; // m
    private static final double MAX_LINEAR_SPEED = 0.25; // m/s; faster than this and camera/odometry timing errors matter
    private static final double MAX_ANGULAR_SPEED = 0.5; // rad/s
    private static final int MIN_SAMPLES = 20;

    private final Drive drive;
    private final PoseEstimator poseEstimator;
    private final int tagId;
    private final Transform3d tagToRobot;
    private final double durationSeconds;

    private final List<List<Transform3d>> samples = new ArrayList<>();
    private final int[] rejectedFrames;
    private Pose2d startOdometryPose;
    private double startTime;

    /**
     * @param tagId the AprilTag to use
     * @param tagToRobot where the robot is relative to the tag, in the tag's frame (see class comment, or use {@link #robotFacingTag})
     * @param durationSeconds how long to collect frames for (3-5 s is plenty at 30 fps)
     */
    public CalibrateCameras(Drive drive, PoseEstimator poseEstimator, int tagId, Transform3d tagToRobot, double durationSeconds) {
        this.drive = drive;
        this.poseEstimator = poseEstimator;
        this.tagId = tagId;
        this.tagToRobot = tagToRobot;
        this.durationSeconds = durationSeconds;
        this.rejectedFrames = new int[poseEstimator.getVision().getCameraCount()];
        // no subsystem requirements on purpose: you can still drive (slowly) while it runs
    }

    /**
     * Builds tag->robot for the common setup: robot in front of the tag, facing it.
     *
     * @param distance from the tag's face to the robot's CENTER, straight out from the tag
     * @param sidewaysOffset robot center's offset to the right (as you look at the tag); usually 0
     * @param yawOffset how far the robot is turned from facing the tag head-on (counterclockwise positive, seen from above); usually 0
     * @param tagCenterHeight height of the tag's center above the floor
     */
    public static Transform3d robotFacingTag(Distance distance, Distance sidewaysOffset, Angle yawOffset, Distance tagCenterHeight) {
        return new Transform3d(
            distance.in(Meters),
            sidewaysOffset.in(Meters),
            -tagCenterHeight.in(Meters),
            new Rotation3d(0, 0, Math.PI + yawOffset.in(Radians)) // facing the tag means facing the tag frame's -X
        );
    }

    @Override
    public void initialize() {
        samples.clear();
        for (int i = 0; i < rejectedFrames.length; i++) {
            samples.add(new ArrayList<>());
            rejectedFrames[i] = 0;
        }
        startOdometryPose = poseEstimator.getOdometryPose();
        startTime = Timer.getFPGATimestamp();
        System.out.printf("[CalibrateCameras] collecting tag %d for %.1f s, keep the robot still...%n", tagId, durationSeconds);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = drive.getRobotRelativeSpeeds();
        boolean tooFast = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > MAX_LINEAR_SPEED
            || Math.abs(speeds.omegaRadiansPerSecond) > MAX_ANGULAR_SPEED;

        for (int camera = 0; camera < samples.size(); camera++) {
            for (TagObservation obs : poseEstimator.getVision().getTagObservations(camera)) {
                if (obs.tagId() != tagId) {
                    continue;
                }
                Optional<Pose2d> odometryPose = poseEstimator.getOdometryPoseAt(obs.timestamp());
                if (tooFast
                    || obs.timestamp() < startTime + SETTLE_TIME
                    || obs.ambiguity() > MAX_AMBIGUITY
                    || obs.cameraToTag().getTranslation().getNorm() > MAX_TAG_DISTANCE
                    || odometryPose.isEmpty()
                ) {
                    rejectedFrames[camera]++;
                    continue;
                }

                // where the robot is relative to the tag at the moment of this frame (start placement + how far odometry says it moved since)
                Transform2d moved = odometryPose.get().minus(startOdometryPose);
                Transform3d tagToRobotNow = tagToRobot.plus(new Transform3d(
                    new Translation3d(moved.getX(), moved.getY(), 0),
                    new Rotation3d(0, 0, moved.getRotation().getRadians())
                ));

                samples.get(camera).add(tagToRobotNow.inverse().plus(obs.cameraToTag().inverse()));
            }
        }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= durationSeconds;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[CalibrateCameras] results" + (interrupted ? " (interrupted early)" : "") + ":");
        for (int camera = 0; camera < samples.size(); camera++) {
            String name = camera < VisionConstants.cameraNames.length ? VisionConstants.cameraNames[camera] : "camera" + camera;
            List<Transform3d> cameraSamples = samples.get(camera);

            if (cameraSamples.size() < MIN_SAMPLES) {
                System.out.printf("  camera %d (%s): only %d good frames of tag %d (%d rejected), need %d. Make sure this camera can see the tag clearly, then run again.%n",
                    camera, name, cameraSamples.size(), tagId, rejectedFrames[camera], MIN_SAMPLES);
                continue;
            }

            // average, throw out outliers, average again
            Transform3d mean = average(cameraSamples);
            List<Transform3d> kept = removeOutliers(cameraSamples, mean);
            mean = average(kept);
            double[] spread = spread(kept, mean);

            Rotation3d r = mean.getRotation();
            System.out.printf("  camera %d (%s): %d frames used (%d outliers, %d rejected), spread %.1f cm / %.2f deg%n",
                camera, name, kept.size(), cameraSamples.size() - kept.size(), rejectedFrames[camera], spread[0] * 100, spread[1]);
            System.out.printf(
                "    new Transform3d(Inches.of(%.2f), Inches.of(%.2f), Inches.of(%.2f), new Rotation3d(Degrees.of(%.1f), Degrees.of(%.1f), Degrees.of(%.1f))),%n",
                Meters.of(mean.getX()).in(Inches), Meters.of(mean.getY()).in(Inches), Meters.of(mean.getZ()).in(Inches),
                Math.toDegrees(r.getX()), Math.toDegrees(r.getY()), Math.toDegrees(r.getZ())
            );

            if (camera < VisionConstants.robotToCameraTransforms.length) {
                Transform3d current = VisionConstants.robotToCameraTransforms[camera];
                double translationDiff = current.getTranslation().getDistance(mean.getTranslation());
                double rotationDiff = Math.toDegrees(angleBetween(current.getRotation(), mean.getRotation()));
                System.out.printf("    differs from current VisionConstants value by %.2f in and %.1f deg%n", Meters.of(translationDiff).in(Inches), rotationDiff);
            }

            Logger.recordOutput("outputs/cameraCalibration/camera" + camera + "/robotToCamera", mean);
            Logger.recordOutput("outputs/cameraCalibration/camera" + camera + "/samples", kept.size());
            Logger.recordOutput("outputs/cameraCalibration/camera" + camera + "/spreadCm", spread[0] * 100);
            Logger.recordOutput("outputs/cameraCalibration/camera" + camera + "/spreadDeg", spread[1]);
            Logger.recordOutput("outputs/cameraCalibration/camera" + camera + "/cameraPose", new Pose3d(poseEstimator.getPose()).plus(mean)); // view this in AdvantageScope's 3D field
        }
    }

    // ————— math helpers ————— //

    private static Transform3d average(List<Transform3d> transforms) {
        double x = 0, y = 0, z = 0;
        double qw = 0, qx = 0, qy = 0, qz = 0;
        Quaternion reference = transforms.get(0).getRotation().getQuaternion();
        for (Transform3d t : transforms) {
            x += t.getX();
            y += t.getY();
            z += t.getZ();

            // q and -q are the same rotation, so flip each one to the same side as the first before adding
            Quaternion q = t.getRotation().getQuaternion();
            double sign = q.dot(reference) < 0 ? -1 : 1;
            qw += sign * q.getW();
            qx += sign * q.getX();
            qy += sign * q.getY();
            qz += sign * q.getZ();
        }
        int n = transforms.size();
        return new Transform3d(new Translation3d(x / n, y / n, z / n), new Rotation3d(new Quaternion(qw, qx, qy, qz).normalize()));
    }

    private static List<Transform3d> removeOutliers(List<Transform3d> transforms, Transform3d mean) {
        List<Double> translationErrors = new ArrayList<>(), rotationErrors = new ArrayList<>();
        for (Transform3d t : transforms) {
            translationErrors.add(t.getTranslation().getDistance(mean.getTranslation()));
            rotationErrors.add(angleBetween(t.getRotation(), mean.getRotation()));
        }
        double translationLimit = Math.max(3 * median(translationErrors), 0.01); // m
        double rotationLimit = Math.max(3 * median(rotationErrors), Math.toRadians(0.5));

        List<Transform3d> kept = new ArrayList<>();
        for (int i = 0; i < transforms.size(); i++) {
            if (translationErrors.get(i) <= translationLimit && rotationErrors.get(i) <= rotationLimit) {
                kept.add(transforms.get(i));
            }
        }
        return kept.size() >= MIN_SAMPLES / 2 ? kept : transforms;
    }

    // RMS distance (m) and angle (deg) of the samples from the mean
    private static double[] spread(List<Transform3d> transforms, Transform3d mean) {
        double translation = 0, rotation = 0;
        for (Transform3d t : transforms) {
            translation += Math.pow(t.getTranslation().getDistance(mean.getTranslation()), 2);
            rotation += Math.pow(Math.toDegrees(angleBetween(t.getRotation(), mean.getRotation())), 2);
        }
        return new double[] {Math.sqrt(translation / transforms.size()), Math.sqrt(rotation / transforms.size())};
    }

    // angle (rad) of the rotation between a and b, in [0, pi]. Rotation3d.getAngle() can return ~2pi for a near-zero rotation (q vs -q)
    private static double angleBetween(Rotation3d a, Rotation3d b) {
        double angle = a.minus(b).getAngle();
        return Math.min(angle, 2 * Math.PI - angle);
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return sorted.get(sorted.size() / 2);
    }
}
