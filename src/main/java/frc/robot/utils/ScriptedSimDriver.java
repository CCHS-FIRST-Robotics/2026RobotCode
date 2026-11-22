/**
 * Original code
 */

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.RobotContainer;
import frc.robot.commands.CalibrateCameras;
import frc.robot.subsystems.poseEstimator.PoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import org.littletonrobotics.junction.Logger;
import java.util.List;
import java.util.function.Consumer;

/**
 * Drives the robot in simulation from a fixed, timed script instead of a real controller,
 * so a full auto + teleop run can be done hands-off and the resulting log analyzed.
 *
 * Only runs when the sim is launched with: ./gradlew simulateJava -PscriptedSim
 * (that flag also turns off the sim GUI, whose driver station would fight this script for control).
 * When the script finishes, the logger is flushed and the program exits.
 *
 * The current step name is logged to "outputs/scriptedSim/step" so the analyzer can split the log by step.
 */
public class ScriptedSimDriver {
    private enum Mode {
        DISABLED,
        AUTO,
        TELEOP
    }

    private record Step(String name, double seconds, Mode mode, Consumer<XboxControllerSim> inputs, Runnable onStart) {
        Step(String name, double seconds, Mode mode, Consumer<XboxControllerSim> inputs) {
            this(name, seconds, mode, inputs, () -> {});
        }
    }

    private static final Consumer<XboxControllerSim> NO_INPUT = c -> {};

    // camera calibration check (-PscriptedSim=calibrate): places the robot in front of a field tag for each camera and runs
    // CalibrateCameras, which should print transforms matching VisionConstants.robotToCameraTransforms (the sim cameras use those)
    private static List<Step> calibrationScript(RobotContainer robotContainer) {
        return List.of(
            new Step("disabled (start)", 1, Mode.DISABLED, NO_INPUT),
            new Step("teleop idle", 1, Mode.TELEOP, NO_INPUT),
            // camera 1 faces forward: robot 2 m in front of tag 26 (blue hub face), facing it
            new Step("calibrate on tag 26 (front camera)", 5.5, Mode.TELEOP, NO_INPUT, () -> CommandScheduler.getInstance().schedule(
                robotContainer.getSimCalibrationTestCommand(26, CalibrateCameras.robotFacingTag(Meters.of(2), Meters.of(0), Degrees.of(0), Meters.of(0)))
            )),
            // camera 0 faces backward (yaw 200 deg): robot 2 m in front of tag 32 (blue alliance wall), turned so that camera faces the tag
            new Step("calibrate on tag 32 (rear camera)", 5.5, Mode.TELEOP, NO_INPUT, () -> CommandScheduler.getInstance().schedule(
                robotContainer.getSimCalibrationTestCommand(32, CalibrateCameras.robotFacingTag(Meters.of(2), Meters.of(0), Degrees.of(160), Meters.of(0)))
            )),
            new Step("disabled (end)", 1, Mode.DISABLED, NO_INPUT)
        );
    }

    // the script. pose is reset to (3, 3) facing +x (blue alliance) at the start of teleop, so all
    // teleop moves are kept inside the blue alliance zone, away from the hub and walls.
    // joystick reminders: leftY = -1 is forward (+x), leftX = -1 is left (+y), rightX = +1 spins clockwise
    // auto start helper check (-PscriptedSim=autostart): places the robot off, then exactly on, an auto's start and prints what the helper says
    private static List<Step> autoStartScript(RobotContainer robotContainer) {
        Runnable print = () -> System.out.println("[ScriptedSimDriver]   autoStart status: \""
            + SmartDashboard.getString("smartDashboard/autoStart/Status", "?") + "\", in position: "
            + SmartDashboard.getBoolean("smartDashboard/autoStart/In Position", false));
        return List.of(
            new Step("disabled (start)", 1, Mode.DISABLED, NO_INPUT),
            new Step("CenterFuelLeft, robot 20 cm back and 5 deg off", 4, Mode.DISABLED, NO_INPUT,
                () -> robotContainer.simPlaceAtAutoStart("CenterFuelLeft", new Transform2d(-0.20, 0, Rotation2d.fromDegrees(5)))),
            new Step("check", 0.1, Mode.DISABLED, NO_INPUT, print),
            new Step("CenterFuelLeft, robot 8 cm right", 4, Mode.DISABLED, NO_INPUT,
                () -> robotContainer.simPlaceAtAutoStart("CenterFuelLeft", new Transform2d(0, -0.08, new Rotation2d()))),
            new Step("check", 0.1, Mode.DISABLED, NO_INPUT, print),
            new Step("CenterFuelLeft, robot exactly on start", 4, Mode.DISABLED, NO_INPUT,
                () -> robotContainer.simPlaceAtAutoStart("CenterFuelLeft", new Transform2d())),
            new Step("check", 0.1, Mode.DISABLED, NO_INPUT, print),
            new Step("BackUpAndShoot selected", 1, Mode.DISABLED, NO_INPUT, () -> robotContainer.simSelectAuto("BackUpAndShoot")),
            new Step("check", 0.1, Mode.DISABLED, NO_INPUT, print),
            new Step("auto (should run normally)", 3, Mode.AUTO, NO_INPUT),
            new Step("disabled (end)", 1, Mode.DISABLED, NO_INPUT)
        );
    }

    // smart auto start check (-PscriptedSim=autooffset): runs CenterFuelLeft from different misplacements, with smart start on and off
    // ("OLD"). the analyzer's AUTO PATH TRACKING section shows how far the robot was from the path, and for how long;
    // ./gradlew analyzeLog -Pexport=some/folder also writes that as CSV
    private static List<Step> autoOffsetScript(RobotContainer robotContainer) {
        record Trial(String name, Transform2d offset, boolean smart) {}
        return trialSteps(robotContainer, List.of(
            new Trial("on start", new Transform2d(), true),
            new Trial("10 cm back", new Transform2d(-0.10, 0, new Rotation2d()), true),
            new Trial("20 cm right, 5 deg", new Transform2d(0, -0.20, Rotation2d.fromDegrees(5)), true),
            new Trial("20 cm right, 5 deg, OLD", new Transform2d(0, -0.20, Rotation2d.fromDegrees(5)), false),
            new Trial("45 cm back", new Transform2d(-0.45, 0, new Rotation2d()), true),
            new Trial("45 cm back, OLD", new Transform2d(-0.45, 0, new Rotation2d()), false),
            new Trial("2 m back (too far)", new Transform2d(-2.0, 0, new Rotation2d()), true)
        ).stream().map(t -> new Object[] {t.name(), t.offset(), t.smart()}).toList(), false);
    }

    // path following check (-PscriptedSim=pathcompare): runs several autos from exactly on their start, to compare how well the
    // robot follows different versions of the paths (e.g. regenerated with a different max torque in Choreo)
    private static List<Step> pathCompareScript(RobotContainer robotContainer) {
        List<Step> steps = new java.util.ArrayList<>();
        steps.add(new Step("disabled (start)", 1, Mode.DISABLED, NO_INPUT, () -> AutoGenerator.simTeleportOnOdometryReset = false));
        for (String auto : List.of("BeMeanLeft", "RepeatingCenterFuelLeftCloser", "CenterFuelLeft")) {
            steps.add(new Step("place: " + auto, 3, Mode.DISABLED, NO_INPUT, () -> robotContainer.simPlaceAtAutoStart(auto, new Transform2d())));
            steps.add(new Step("auto: " + auto, 6, Mode.AUTO, NO_INPUT));
            steps.add(new Step("disabled", 0.5, Mode.DISABLED, NO_INPUT));
        }
        steps.add(new Step("restore", 0.1, Mode.DISABLED, NO_INPUT, () -> AutoGenerator.simTeleportOnOdometryReset = true));
        return steps;
    }

    // no vision check (-PscriptedSim=autonovision): same as above, but the pose estimator ignores the cameras the whole time,
    // like a camera being unplugged or no tags in view. smart start should fall back to resetting odometry
    private static List<Step> autoNoVisionScript(RobotContainer robotContainer) {
        return trialSteps(robotContainer, List.of(
            new Object[] {"no vision, on start", new Transform2d(), true},
            new Object[] {"no vision, 20 cm right, 5 deg", new Transform2d(0, -0.20, Rotation2d.fromDegrees(5)), true},
            new Object[] {"no vision, 45 cm back", new Transform2d(-0.45, 0, new Rotation2d()), true}
        ), true);
    }

    // each trial: place the robot off the CenterFuelLeft start (disabled), run the auto, then disable (the sim resets the robot)
    private static List<Step> trialSteps(RobotContainer robotContainer, List<Object[]> trials, boolean noVision) {
        Runnable print = () -> System.out.println("[ScriptedSimDriver]   autoStart status: \""
            + SmartDashboard.getString("smartDashboard/autoStart/Status", "?") + "\"");
        List<Step> steps = new java.util.ArrayList<>();
        steps.add(new Step("disabled (start)", 1, Mode.DISABLED, NO_INPUT, () -> {
            AutoGenerator.simTeleportOnOdometryReset = false; // so resetting odometry can't cheat by moving the sim robot onto the path
            PoseEstimator.simIgnoreVision = noVision;
        }));
        for (Object[] trial : trials) {
            String name = (String) trial[0];
            steps.add(new Step("place: " + name, 3, Mode.DISABLED, NO_INPUT, () -> {
                SmartDashboard.putBoolean("smartDashboard/toggles/Enable Smart Auto Start", (Boolean) trial[2]);
                robotContainer.simPlaceAtAutoStart("CenterFuelLeft", (Transform2d) trial[1]);
            }));
            steps.add(new Step("check", 0.1, Mode.DISABLED, NO_INPUT, print));
            steps.add(new Step("auto: " + name, 5, Mode.AUTO, NO_INPUT));
            steps.add(new Step("disabled", 0.5, Mode.DISABLED, NO_INPUT));
        }
        steps.add(new Step("restore", 0.1, Mode.DISABLED, NO_INPUT, () -> {
            SmartDashboard.putBoolean("smartDashboard/toggles/Enable Smart Auto Start", true);
            AutoGenerator.simTeleportOnOdometryReset = true;
            PoseEstimator.simIgnoreVision = false;
        }));
        return steps;
    }

    private static final List<Step> DRIVE_SCRIPT = List.of(
        new Step("disabled (start)", 2, Mode.DISABLED, NO_INPUT),
        new Step("auto (selected routine)", 10, Mode.AUTO, NO_INPUT),
        new Step("disabled (after auto)", 2, Mode.DISABLED, NO_INPUT),

        // reversing steps are longer: the drive is current limited (~60 A), so a full reversal takes about 1 s
        new Step("teleop idle", 1, Mode.TELEOP, NO_INPUT),
        new Step("drive backward", 1, Mode.TELEOP, c -> c.setLeftY(1)),
        new Step("drive forward", 1.5, Mode.TELEOP, c -> c.setLeftY(-1)),
        new Step("strafe left", 1, Mode.TELEOP, c -> c.setLeftX(-1)),
        new Step("strafe right", 1.5, Mode.TELEOP, c -> c.setLeftX(1)),
        new Step("stop", 0.75, Mode.TELEOP, NO_INPUT),
        new Step("spin in place", 2, Mode.TELEOP, c -> c.setRightX(1)),
        new Step("drive backward while turning", 2, Mode.TELEOP, c -> {
            c.setLeftY(0.5);
            c.setRightX(-0.5);
        }),
        new Step("settle", 1, Mode.TELEOP, NO_INPUT),

        // toggled twice so the pivot ends where it started (the auto's shoot command leaves it down, and the intake only runs when it's down)
        new Step("toggle pivot (1)", 0.25, Mode.TELEOP, c -> c.setLeftBumperButton(true)),
        new Step("wait for pivot (1)", 1.25, Mode.TELEOP, NO_INPUT),
        new Step("toggle pivot (2)", 0.25, Mode.TELEOP, c -> c.setLeftBumperButton(true)),
        new Step("wait for pivot (2)", 1.25, Mode.TELEOP, NO_INPUT),
        new Step("drive and intake", 3, Mode.TELEOP, c -> {
            c.setLeftTriggerAxis(1);
            c.setLeftY(-0.4);
        }),

        new Step("x-lock", 1.5, Mode.TELEOP, c -> c.setXButton(true)),
        new Step("drive and shoot (aim at hub)", 5, Mode.TELEOP, c -> c.setRightTriggerAxis(1)),
        new Step("let shooter spin down", 1.5, Mode.TELEOP, NO_INPUT),
        new Step("unstick", 2, Mode.TELEOP, c -> c.setPOV(180)),

        new Step("disabled (end)", 1, Mode.DISABLED, NO_INPUT)
    );

    private final XboxControllerSim controller;
    private final List<Step> SCRIPT;

    private int stepIndex = 0;
    private double stepStartTime = -1;

    public static boolean isRequested() {
        return System.getenv("SCRIPTED_SIM") != null;
    }

    public ScriptedSimDriver(int controllerPort, RobotContainer robotContainer) {
        controller = new XboxControllerSim(controllerPort); // also tells the sim an xbox controller is plugged in
        SCRIPT = switch (String.valueOf(System.getenv("SCRIPTED_SIM"))) {
            case "calibrate" -> calibrationScript(robotContainer);
            case "autostart" -> autoStartScript(robotContainer);
            case "autooffset" -> autoOffsetScript(robotContainer);
            case "autonovision" -> autoNoVisionScript(robotContainer);
            case "pathcompare" -> pathCompareScript(robotContainer);
            default -> DRIVE_SCRIPT;
        };

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();

        double total = SCRIPT.stream().mapToDouble(Step::seconds).sum();
        System.out.println("[ScriptedSimDriver] running " + SCRIPT.size() + " steps (" + total + " s)");
    }

    // call once per loop from simulationPeriodic
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        if (stepStartTime < 0) {
            stepStartTime = now;
            printStep();
            SCRIPT.get(stepIndex).onStart().run();
        }

        // advance to the next step when the current one has run long enough
        if (now - stepStartTime >= SCRIPT.get(stepIndex).seconds()) {
            stepIndex++;
            stepStartTime = now;

            if (stepIndex >= SCRIPT.size()) {
                finish();
                return;
            }
            printStep();
            SCRIPT.get(stepIndex).onStart().run();
        }

        Step step = SCRIPT.get(stepIndex);

        // inputs are fully rewritten every loop, so nothing carries over from the previous step
        clearInputs();
        step.inputs().accept(controller);
        controller.notifyNewData();

        DriverStationSim.setAutonomous(step.mode() == Mode.AUTO);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(step.mode() != Mode.DISABLED);
        DriverStationSim.notifyNewData();

        Logger.recordOutput("outputs/scriptedSim/step", step.name());
        Logger.recordOutput("outputs/scriptedSim/stepIndex", stepIndex);
    }

    private void clearInputs() {
        controller.setLeftX(0);
        controller.setLeftY(0);
        controller.setRightX(0);
        controller.setRightY(0);
        controller.setLeftTriggerAxis(0);
        controller.setRightTriggerAxis(0);
        controller.setAButton(false);
        controller.setBButton(false);
        controller.setXButton(false);
        controller.setYButton(false);
        controller.setLeftBumperButton(false);
        controller.setRightBumperButton(false);
        controller.setLeftStickButton(false);
        controller.setRightStickButton(false);
        controller.setBackButton(false);
        controller.setStartButton(false);
        controller.setPOV(-1);
    }

    private void printStep() {
        Step step = SCRIPT.get(stepIndex);
        System.out.println("[ScriptedSimDriver] step " + stepIndex + ": " + step.name() + " (" + step.mode() + ", " + step.seconds() + " s)");
    }

    private void finish() {
        System.out.println("[ScriptedSimDriver] script complete, flushing log and exiting");
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        Logger.end(); // flushes and closes the .wpilog
        System.exit(0);
    }
}
