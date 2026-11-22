# 2026RobotCode

## To do:
 - hehe, nothing
## Scripted sim testing
Runs the robot in sim with a scripted "driver" (auto, then a teleop sequence of driving, intaking, x-lock, shooting and unsticking), saves a `.wpilog`, and checks it for problems.

```
./gradlew simulateJava -PscriptedSim    # ~50 s, headless, writes logs/akit_*.wpilog
./gradlew analyzeSimLog                 # report on the newest log in logs/
```

- The script lives in `src/main/java/frc/robot/utils/ScriptedSimDriver.java`; it only runs with `-PscriptedSim`, never on the real robot. The auto it runs is whatever the AutoChooser defaults to.
- The analyzer lives in `src/tools/java/SimLogAnalyzer.java` (not deployed to the robot). Thresholds are at the top of the file.
- `./gradlew analyzeSimLog -Plog=logs/some.wpilog` analyzes a specific log (works on real robot logs too, minus the checks that need the sim's true pose).
- `./gradlew analyzeSimLog -Plist` lists every logged key; `-Pdump=key1,key2 -Pfrom=5 -Pto=8 -Pevery=5` prints values loop by loop. In Git Bash, set `MSYS_NO_PATHCONV=1` first so keys starting with `/` aren't mangled.
- Every sim run (scripted or not) now also writes a `.wpilog` to `logs/`, so you can open it in AdvantageScope afterwards.
- If Gradle can't find Java, set `JAVA_HOME` to WPILib's JDK (`C:\Users\Public\wpilib\2026\jdk`).

## Camera calibration (no tape measure for the cameras)
`CalibrateCameras` works out each camera's `robotToCameraTransforms` entry from what the camera sees. The only things you measure are where the robot is relative to one AprilTag and the tag's height.

1. Put the robot in front of a tag, facing it (or turned, for a rear camera), and measure the distance from the tag's face to the robot's **center**.
2. On the dashboard, under `cameraCalibration`, fill in the tag ID, that distance, any sideways offset or turn, and the tag center height (0 = use the official field layout height).
3. Press **Calibrate Cameras** (works while disabled) and keep the robot still for 4 s.
4. Each camera that saw the tag prints a `new Transform3d(...)` line to the console. Paste it into `VisionConstants.robotToCameraTransforms`. Results are also logged under `outputs/cameraCalibration/`, including a `cameraPose` you can view in AdvantageScope's 3D field.
5. Cameras facing different ways need separate runs: re-place the robot so the other camera sees a tag.

The tag doesn't have to be on a real field; any tag printed and taped to a wall works if you enter its height. `./gradlew simulateJava -PscriptedSim=calibrate` checks the math in sim: it should print values within about 0.5 in and 0.5 deg of the current `VisionConstants`.
