/**
 * Original code
 */

package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;
import java.util.Optional;
import frc.robot.Constants;
import frc.robot.subsystems.leds.LedStrip;
import frc.robot.subsystems.poseEstimator.PoseEstimator;

/**
 * Helps place the robot at the selected auto's starting spot before a match.
 *
 * Choreo autos start with resetOdometry(), which tells the robot it's exactly at the path's start no matter where it
 * really is, so a robot placed 20 cm off runs the whole auto 20 cm off. While disabled, this compares where vision says
 * the robot is to where the auto starts, and shows the result:
 *   LEDs:      green = in position, yellow = close, red = far off, rainbow (normal idle) = can't tell (no vision, or the
 *              selected auto doesn't start from a fixed spot)
 *   Dashboard: smartDashboard/autoStart/Status says which way to move the robot, in the robot's own directions
 *              (forward = the way the robot's front faces), so you can push it into place
 *   Field:     an "autoStart" robot outline on the field2d marks the target spot
 */
public class AutoStartHelper {
    // ————— tuning ————— //

    // (also used by AutoGenerator.smartStart to decide how to start an auto)
    public static final double ALIGNED_TRANSLATION = 0.05; // m
    public static final double ALIGNED_ROTATION = Math.toRadians(2);
    public static final double CLOSE_TRANSLATION = 0.30; // m
    public static final double CLOSE_ROTATION = Math.toRadians(10);
    public static final double MAX_SECONDS_SINCE_VISION = 1.0; // older than this and the pose can't be trusted

    private static final int GREEN = 60, YELLOW = 30, RED = 0; // LED hues (0-180)

    private enum State {
        UNKNOWN,
        FAR,
        CLOSE,
        ALIGNED
    }

    private final PoseEstimator poseEstimator;
    private final LedStrip ledStrip;

    private State shownState = null;

    public AutoStartHelper(PoseEstimator poseEstimator, LedStrip ledStrip) {
        this.poseEstimator = poseEstimator;
        this.ledStrip = ledStrip;
    }

    // call every loop while disabled
    public void update(Optional<Pose2d> autoStartPose) {
        State state;
        String status;
        double translationError = -1, rotationErrorDeg = -1;

        if (autoStartPose.isEmpty()) {
            state = State.UNKNOWN;
            status = "Selected auto doesn't start from a fixed spot";
            Constants.FieldConstants.FIELD2D.getObject("autoStart").setPoses();
        } else {
            Constants.FieldConstants.FIELD2D.getObject("autoStart").setPose(autoStartPose.get());

            if (poseEstimator.getSecondsSinceVision() > MAX_SECONDS_SINCE_VISION) {
                state = State.UNKNOWN;
                status = "No vision, can't check position (make sure a camera can see a tag)";
            } else {
                // the start pose as seen from the robot: x = how far forward to move, y = how far left, rotation = how far to turn CCW
                Transform2d move = new Transform2d(poseEstimator.getPose(), autoStartPose.get());
                translationError = move.getTranslation().getNorm();
                double rotationError = Math.abs(move.getRotation().getRadians());
                rotationErrorDeg = Math.toDegrees(rotationError);

                if (translationError <= ALIGNED_TRANSLATION && rotationError <= ALIGNED_ROTATION) {
                    state = State.ALIGNED;
                    status = "IN POSITION";
                } else {
                    state = translationError <= CLOSE_TRANSLATION && rotationError <= CLOSE_ROTATION ? State.CLOSE : State.FAR;
                    status = describeMove(move);
                }
            }
        }

        SmartDashboard.putString("smartDashboard/autoStart/Status", status);
        SmartDashboard.putBoolean("smartDashboard/autoStart/In Position", state == State.ALIGNED);
        Logger.recordOutput("outputs/autoStart/state", state);
        Logger.recordOutput("outputs/autoStart/translationErrorMeters", translationError);
        Logger.recordOutput("outputs/autoStart/rotationErrorDegrees", rotationErrorDeg);
        autoStartPose.ifPresent(pose -> Logger.recordOutput("outputs/autoStart/targetPose", pose));

        // only touch the LEDs when the state changes, so the LED timer isn't restarted every loop
        if (state != shownState) {
            ledStrip.setLedStripHues(switch (state) {
                case ALIGNED -> new Integer[] {GREEN};
                case CLOSE -> new Integer[] {YELLOW};
                case FAR -> new Integer[] {RED};
                case UNKNOWN -> new Integer[0]; // normal rainbow idle
            });
            shownState = state;
        }
    }

    // call when the robot is enabled, to hand the LEDs back to the rest of the code
    public void stop() {
        if (shownState != null) {
            ledStrip.setLedStripHues(new Integer[0]);
            shownState = null;
        }
        Constants.FieldConstants.FIELD2D.getObject("autoStart").setPoses();
    }

    // e.g. "Move 12 cm forward, 3 cm right, turn 4 deg left"
    private static String describeMove(Transform2d move) {
        StringBuilder s = new StringBuilder("Move ");
        double forward = move.getX() * 100, left = move.getY() * 100, turn = move.getRotation().getDegrees();
        s.append(String.format("%.0f cm %s", Math.abs(forward), forward >= 0 ? "forward" : "back"));
        s.append(String.format(", %.0f cm %s", Math.abs(left), left >= 0 ? "left" : "right"));
        s.append(String.format(", turn %.0f deg %s", Math.abs(turn), turn >= 0 ? "left (CCW)" : "right (CW)"));
        return s.append(" (robot's directions)").toString();
    }
}
