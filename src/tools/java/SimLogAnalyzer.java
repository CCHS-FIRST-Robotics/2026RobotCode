import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.File;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.*;
import java.util.function.Function;

/**
 * Reads an AdvantageKit .wpilog (normally from ./gradlew simulateJava -PscriptedSim) and checks it for problems:
 * errors/alerts, loop timing, drive tracking, pose estimation vs. the sim's true pose, mechanism setpoint tracking,
 * motor currents, and battery voltage. Prints a report and a list of anything that looks wrong.
 *
 * See build.gradle for how to run it. Thresholds are at the top of the file; tune them as you learn what's normal.
 */
public class SimLogAnalyzer {
    // ————— thresholds ————— //

    static final double LOOP_BUDGET_MS = 20;
    static final double MAX_OVERRUN_FRACTION = 0.05; // fraction of enabled loops allowed over budget
    static final double MAX_LINEAR_TRACKING_ERROR = 0.25; // m/s, mean over the second half of a step
    static final double MAX_ANGULAR_TRACKING_ERROR = 0.5; // rad/s
    static final double MAX_MODULE_ANGLE_ERROR_DEG = 10; // mean, only while the module is moving
    static final double MAX_COMBINED_POSE_ERROR = 0.30; // m, mean while enabled
    static final double MAX_ODOMETRY_POSE_ERROR = 0.50; // m, max while enabled
    static final double MAX_HEADING_ERROR_DEG = 5;
    static final double POSE_JUMP = 0.30; // m in a single loop
    static final double SHOOTER_SPINUP_LIMIT = 1.5; // s
    static final double PIVOT_TOLERANCE = 0.02; // rotations
    static final double PIVOT_SETTLE_LIMIT = 1.5; // s
    static final double MIN_BATTERY_VOLTAGE = 9.0;
    static final double TRANSITION_IGNORE = 0.25; // s ignored after enable/disable/reset when checking poses

    // ————— keys ————— //

    static final String OUT = "/RealOutputs/outputs/";
    static final String STEP = OUT + "scriptedSim/step";
    static final String TRUE_POSE = OUT + "simulation/fieldSimulation/robotPosition";
    static final String SPEEDS_IN = OUT + "drive/speedsInput";
    static final String SPEEDS_OUT = OUT + "drive/speedsOutput";
    static final String MODULES_IN = OUT + "drive/moduleStatesInput";
    static final String MODULES_OUT = OUT + "drive/moduleStatesOutput";
    static final String POSE_EST = OUT + "poseEstimator/poseEstimates/";
    static final String VISION = OUT + "poseEstimator/vision/";
    static final String SHOOTER = "/subsystems/fuelIO/shooter/shooter/";
    static final String KICKER = "/subsystems/fuelIO/shooter/kicker/";
    static final String PIVOT = "/subsystems/fuelIO/intake/pivot/";
    static final String INTAKE = "/subsystems/fuelIO/intake/intake/";

    // ————— log loading ————— //

    record Sample(double t, Object value) {}

    static final Map<String, String> types = new TreeMap<>();
    static final Map<String, List<Sample>> data = new HashMap<>();
    static double logStart = Double.MAX_VALUE, logEnd = 0;

    static void load(File file) throws Exception {
        DataLogReader reader = new DataLogReader(file.getPath());
        if (!reader.isValid()) {
            throw new IllegalArgumentException("not a valid .wpilog: " + file);
        }

        Map<Integer, String> entryNames = new HashMap<>();
        // real robot logs can be cut off (robot powered off mid-write) or damaged, so: skip records that don't decode,
        // and stop cleanly (keeping everything read so far) if the file itself breaks
        Map<Integer, String> entryTypes = new HashMap<>();
        Iterator<DataLogRecord> it = reader.iterator();
        badRecords = 0;
        stoppedEarly = null;
        while (true) {
            DataLogRecord r;
            try {
                if (!it.hasNext()) break;
                r = it.next();
            } catch (RuntimeException e) {
                stoppedEarly = e.getMessage();
                break;
            }
            try {
                if (r.isStart()) {
                    DataLogRecord.StartRecordData start = r.getStartData();
                    entryNames.put(start.entry, start.name);
                    entryTypes.put(start.entry, start.type);
                    types.put(start.name, start.type);
                    data.putIfAbsent(start.name, new ArrayList<>());
                    continue;
                }
                if (r.isControl()) {
                    continue;
                }

                String name = entryNames.get(r.getEntry());
                if (name == null) {
                    continue;
                }
                Object value = switch (entryTypes.get(r.getEntry())) {
                    case "double" -> r.getDouble();
                    case "float" -> (double) r.getFloat();
                    case "int64" -> (double) r.getInteger();
                    case "boolean" -> r.getBoolean();
                    case "string", "json" -> r.getString();
                    case "double[]" -> r.getDoubleArray();
                    case "boolean[]" -> r.getBooleanArray();
                    case "int64[]" -> r.getIntegerArray();
                    case "float[]" -> r.getFloatArray();
                    case "string[]" -> r.getStringArray();
                    default -> r.getRaw(); // structs, decoded on demand
                };
                double t = r.getTimestamp() / 1e6;
                data.get(name).add(new Sample(t, value));
                if (name.equals("/Timestamp")) {
                    logStart = Math.min(logStart, t);
                    logEnd = Math.max(logEnd, t);
                }
            } catch (RuntimeException e) {
                badRecords++;
            }
        }
        if (badRecords > 0 || stoppedEarly != null) {
            System.out.printf("WARNING: %d record(s) couldn't be decoded%s%n", badRecords,
                stoppedEarly == null ? "" : String.format("; the file breaks after %.1f s of log (%s), so the rest is lost", logEnd - logStart, stoppedEarly));
        }
    }

    static int badRecords = 0;
    static String stoppedEarly = null;

    static List<Sample> series(String key) {
        return data.getOrDefault(key, List.of());
    }

    // last value logged at or before t (wpilogs only store changes, so this is "the value at time t")
    static Object at(String key, double t) {
        List<Sample> s = series(key);
        int lo = 0, hi = s.size() - 1, best = -1;
        while (lo <= hi) {
            int mid = (lo + hi) >>> 1;
            if (s.get(mid).t() <= t) {
                best = mid;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        return best < 0 ? null : s.get(best).value();
    }

    // time of the last sample at or before t (-infinity if none)
    static double lastSampleTime(String key, double t) {
        List<Sample> s = series(key);
        int lo = 0, hi = s.size() - 1, best = -1;
        while (lo <= hi) {
            int mid = (lo + hi) >>> 1;
            if (s.get(mid).t() <= t) {
                best = mid;
                lo = mid + 1;
            } else {
                hi = mid - 1;
            }
        }
        return best < 0 ? Double.NEGATIVE_INFINITY : s.get(best).t();
    }

    static double num(String key, double t) {
        Object v = at(key, t);
        if (v instanceof Double d) return d;
        if (v instanceof Boolean b) return b ? 1 : 0;
        return Double.NaN;
    }

    static boolean bool(String key, double t) {
        return Boolean.TRUE.equals(at(key, t));
    }

    // ————— struct decoding (all little-endian doubles) ————— //

    static double[] doubles(Object raw) {
        if (!(raw instanceof byte[] bytes)) return null;
        ByteBuffer b = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN);
        double[] out = new double[bytes.length / 8];
        for (int i = 0; i < out.length; i++) out[i] = b.getDouble();
        return out;
    }

    // Pose2d = x, y, theta. ChassisSpeeds = vx, vy, omega. SwerveModuleState = speed, angle (repeated per module).
    static double[] struct(String key, double t) {
        return doubles(at(key, t));
    }

    static int pose3dCount(String key, double t) {
        Object raw = at(key, t);
        return raw instanceof byte[] bytes ? bytes.length / 56 : 0; // Pose3d = translation (3) + quaternion (4) doubles
    }

    static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    // ————— steps ————— //

    record Step(String name, double start, double end, boolean enabled, boolean auto) {
        double length() { return end - start; }
        double mid() { return (start + end) / 2; }
    }

    static List<Step> cachedSteps = null;

    static List<Step> steps() {
        if (cachedSteps != null) return cachedSteps;
        List<Step> steps = new ArrayList<>();
        List<Sample> marks = matchMode ? List.of() : series(STEP);
        for (int i = 0; i < marks.size(); i++) {
            double start = marks.get(i).t();
            double end = i + 1 < marks.size() ? marks.get(i + 1).t() : logEnd;
            double probe = Math.min(start + 0.1, (start + end) / 2); // DS state applies on the loop after the script sets it
            steps.add(new Step((String) marks.get(i).value(), start, end, bool("/DriverStation/Enabled", probe), bool("/DriverStation/Autonomous", probe)));
        }
        if (steps.isEmpty()) { // not a scripted run (e.g. a real match): split the log wherever the robot was enabled/disabled or switched auto/teleop
            String current = null;
            double start = logStart;
            for (Sample s : series("/Timestamp")) {
                boolean enabled = bool("/DriverStation/Enabled", s.t());
                String mode = !enabled ? "disabled" : bool("/DriverStation/Autonomous", s.t()) ? "auto" : "teleop";
                if (!mode.equals(current)) {
                    if (current != null) steps.add(new Step(current, start, s.t(), !current.equals("disabled"), current.equals("auto")));
                    current = mode;
                    start = s.t();
                }
            }
            if (current != null) steps.add(new Step(current, start, logEnd, !current.equals("disabled"), current.equals("auto")));
        }
        cachedSteps = steps;
        return steps;
    }

    // timestamps of loops (one per robot loop) inside [start, end)
    static List<Double> loops(double start, double end) {
        List<Double> out = new ArrayList<>();
        for (Sample s : series("/Timestamp")) {
            if (s.t() >= start && s.t() < end) out.add(s.t());
        }
        return out;
    }

    static double mean(List<Double> v) {
        return v.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
    }

    static double max(List<Double> v) {
        return v.stream().mapToDouble(Double::doubleValue).max().orElse(Double.NaN);
    }

    static double percentile(List<Double> v, double p) {
        if (v.isEmpty()) return Double.NaN;
        List<Double> sorted = new ArrayList<>(v);
        Collections.sort(sorted);
        return sorted.get((int) Math.min(sorted.size() - 1, Math.floor(p * sorted.size())));
    }

    // ————— report ————— //

    static final List<String> problems = new ArrayList<>();

    static void problem(String msg) {
        problems.add(msg);
        System.out.println("  [PROBLEM] " + msg);
    }

    static void header(String title) {
        System.out.println();
        System.out.println("===== " + title + " =====");
    }

    // real robot / match log (no sim "true pose" to compare against); --match forces this for testing on sim logs
    static boolean matchMode = false;
    static boolean forceMatchMode = false;

    static void reset() {
        types.clear();
        data.clear();
        problems.clear();
        cachedSteps = null;
        logStart = Double.MAX_VALUE;
        logEnd = 0;
    }

    public static void main(String[] args) throws Exception {
        boolean listOnly = Arrays.asList(args).contains("--list");
        forceMatchMode = Arrays.asList(args).contains("--match");
        exportDir = Arrays.stream(args).filter(x -> x.startsWith("--export=")).findFirst().map(x -> x.substring(9)).orElse(null);
        File file = Arrays.stream(args).filter(a -> !a.startsWith("--")).findFirst().map(File::new).orElseGet(SimLogAnalyzer::newestLog);

        if (file.isDirectory()) {
            summarizeFolder(file);
            return;
        }

        System.out.println("Log: " + file.getPath());
        load(file);
        matchMode = forceMatchMode || !data.containsKey(TRUE_POSE);

        if (listOnly) {
            types.forEach((name, type) -> System.out.printf("%-90s %-32s %d samples%n", name, type, data.get(name).size()));
            return;
        }

        // --dump=key1,key2 [--from=seconds] [--to=seconds] [--every=N] prints values loop by loop (times are seconds since log start)
        Optional<String> dump = Arrays.stream(args).filter(a -> a.startsWith("--dump=")).findFirst();
        if (dump.isPresent()) {
            String[] keys = dump.get().substring(7).split(",");
            double from = Arrays.stream(args).filter(a -> a.startsWith("--from=")).findFirst().map(a -> Double.parseDouble(a.substring(7))).orElse(0.0);
            double to = Arrays.stream(args).filter(a -> a.startsWith("--to=")).findFirst().map(a -> Double.parseDouble(a.substring(5))).orElse(1e9);
            int every = Arrays.stream(args).filter(a -> a.startsWith("--every=")).findFirst().map(a -> Integer.parseInt(a.substring(8))).orElse(1);
            List<Step> steps = steps();
            int i = 0;
            for (double t : loops(logStart + from, logStart + to)) {
                if (i++ % every != 0) continue;
                StringBuilder line = new StringBuilder(String.format("%7.2f  %-28s", t - logStart, stepAt(steps, t)));
                for (String key : keys) {
                    Object v = at(key, t);
                    String s = v instanceof byte[] ? Arrays.toString(Arrays.stream(doubles(v)).map(d -> Math.round(d * 1000) / 1000.0).toArray())
                        : v instanceof Double d ? String.format("%.3f", d)
                        : v instanceof double[] arr ? Arrays.toString(arr)
                        : String.valueOf(v);
                    line.append("  ").append(s);
                }
                System.out.println(line);
            }
            return;
        }

        analyze();

        header("SUMMARY");
        if (problems.isEmpty()) {
            System.out.println("No problems found.");
        } else {
            System.out.println(problems.size() + " problem(s):");
            problems.forEach(p -> System.out.println("  - " + p));
        }
    }

    static void analyze() {
        List<Step> steps = steps();
        System.out.printf("Duration: %.1f s, %d loops, %d %s%n", logEnd - logStart, series("/Timestamp").size(), steps.size(),
            matchMode ? "segments (real robot / match log)" : "steps (sim log)");
        if (matchMode) printMatchInfo();

        checkMessages();
        checkLoopTiming(steps);
        checkSteps(steps);
        checkAutoTracking(steps);
        checkModules(steps);
        if (matchMode) {
            checkPoseEstimationMatch(steps);
        } else {
            checkPoseEstimation(steps);
        }
        checkVision(steps);
        checkShooter(steps);
        checkPivotAndIntake(steps);
        checkCurrentsAndBattery(steps);
        checkRobotHealth(steps);
        if (!matchMode) checkSimFidelity();
    }

    static String matchName() {
        Object event = at("/DriverStation/EventName", logEnd), number = at("/DriverStation/MatchNumber", logEnd), type = at("/DriverStation/MatchType", logEnd);
        String[] types = {"none", "practice", "qual", "elim"};
        int typeIndex = type instanceof Double d ? d.intValue() : 0;
        if (typeIndex == 0) return "no FMS match";
        return String.format("%s %s %.0f", event, types[Math.min(typeIndex, 3)], number instanceof Double d ? d : 0);
    }

    static void printMatchInfo() {
        Object station = at("/DriverStation/AllianceStation", logEnd);
        String[] stations = {"unknown", "red 1", "red 2", "red 3", "blue 1", "blue 2", "blue 3"};
        int s = station instanceof Double d ? d.intValue() : 0;
        System.out.printf("Match: %s, %s, FMS attached: %s%n", matchName(), stations[Math.max(0, Math.min(s, 6))], bool("/DriverStation/FMSAttached", logEnd));
    }

    // one line per log in a folder, e.g. every match from an event
    static void summarizeFolder(File folder) throws Exception {
        File[] logs = folder.listFiles((dir, name) -> name.endsWith(".wpilog"));
        if (logs == null || logs.length == 0) {
            System.out.println("No .wpilog files in " + folder);
            return;
        }
        Arrays.sort(logs, Comparator.comparingLong(File::lastModified));
        java.io.PrintStream out = System.out;
        out.printf("%d logs in %s%n%n", logs.length, folder);
        for (File log : logs) {
            reset();
            System.setOut(new java.io.PrintStream(java.io.OutputStream.nullOutputStream()));
            String line;
            try {
                load(log);
                matchMode = forceMatchMode || !data.containsKey(TRUE_POSE);
                double enabledTime = steps().stream().filter(Step::enabled).mapToDouble(Step::length).sum();
                analyze();
                line = String.format("%-36s %-28s enabled %5.1f s  %s", log.getName(), matchName(), enabledTime,
                    problems.isEmpty() ? "OK" : problems.size() + " problem(s)");
            } catch (Exception e) {
                line = String.format("%-36s could not read: %s", log.getName(), e.getMessage());
            } finally {
                System.setOut(out);
            }
            out.println(line);
            for (String p : problems.subList(0, Math.min(5, problems.size()))) out.println("    - " + p);
            if (problems.size() > 5) out.println("    - ... " + (problems.size() - 5) + " more (run on just this log for details)");
        }
    }

    // ————— checks ————— //

    static void checkMessages() {
        header("ERRORS, ALERTS AND CONSOLE");
        for (String key : types.keySet()) {
            if (!(key.endsWith("/errors") || key.endsWith("/warnings")) || !key.startsWith("/RealOutputs/")) continue;
            // group alerts by their start (some alerts grow every loop, e.g. a list of missing files) and note when they were first raised
            Map<String, String> unique = new LinkedHashMap<>();
            Map<String, Double> firstSeen = new HashMap<>();
            for (Sample s : series(key)) {
                if (!(s.value() instanceof String[] arr)) continue;
                for (String msg : arr) {
                    String group = msg.length() > 80 ? msg.substring(0, 80) : msg;
                    unique.put(group, msg.length() > 200 ? msg.substring(0, 200) + "..." : msg);
                    firstSeen.putIfAbsent(group, s.t());
                }
            }
            for (var entry : unique.entrySet()) {
                String text = String.format("%s: %s (first raised t=%.1f s, during '%s')", key.replace("/RealOutputs/", ""), entry.getValue(),
                    firstSeen.get(entry.getKey()) - logStart, stepAt(steps(), firstSeen.get(entry.getKey())));
                if (key.endsWith("/errors")) {
                    problem(text);
                } else {
                    System.out.println("  [warning] " + text);
                }
            }
        }

        int overruns = 0;
        Set<String> consoleIssues = new LinkedHashSet<>();
        for (Sample s : series("/RealOutputs/Console")) {
            for (String line : ((String) s.value()).split("\n")) {
                if (line.contains("overrun")) overruns++;
                else if (line.contains("Exception") || line.contains("Error") || line.contains("ERROR")) consoleIssues.add(line.trim());
                else if (line.startsWith("Warning") && !line.contains("Tracer") && !line.contains("Joystick")) consoleIssues.add(line.trim());
            }
        }
        System.out.println("  console: " + overruns + " loop overrun message(s)");
        consoleIssues.forEach(line -> System.out.println("  [console] " + line));
    }

    static void checkLoopTiming(List<Step> steps) {
        header("LOOP TIMING");
        List<Double> enabled = new ArrayList<>(), disabled = new ArrayList<>();
        for (Sample s : series("/RealOutputs/LoggedRobot/FullCycleMS")) {
            (bool("/DriverStation/Enabled", s.t()) ? enabled : disabled).add((Double) s.value());
        }
        for (var entry : List.of(Map.entry("enabled", enabled), Map.entry("disabled", disabled))) {
            List<Double> v = entry.getValue();
            long over = v.stream().filter(ms -> ms > LOOP_BUDGET_MS).count();
            System.out.printf("  %-8s mean %5.1f ms   p95 %5.1f ms   max %6.1f ms   over %.0f ms: %d/%d loops%n",
                entry.getKey(), mean(v), percentile(v, 0.95), max(v), LOOP_BUDGET_MS, over, v.size());
            if (entry.getKey().equals("enabled") && !v.isEmpty() && (double) over / v.size() > MAX_OVERRUN_FRACTION) {
                problem(String.format("%.0f%% of enabled loops took longer than %.0f ms", 100.0 * over / v.size(), LOOP_BUDGET_MS));
            }
        }
        List<Sample> worst = new ArrayList<>(series("/RealOutputs/LoggedRobot/FullCycleMS"));
        worst.sort(Comparator.comparingDouble(s -> -(Double) s.value()));
        worst.removeIf(s -> stepAt(steps, s.t()).equals("?")); // first loop includes robot program startup
        for (Sample s : worst.subList(0, Math.min(4, worst.size()))) {
            System.out.printf("  slow loop: %6.1f ms at t=%.2f s during '%s' (GC %.0f ms)%n", (Double) s.value(), s.t() - logStart, stepAt(steps, s.t()),
                num("/RealOutputs/LoggedRobot/GCTimeMS", s.t()));
        }
        System.out.println("  (sim loop times include PC-side physics/vision sim, so they're pessimistic vs. the roboRIO for some things and optimistic for others)");
    }

    // expected motion for script steps (matched by name; steps not listed here aren't checked for direction)
    record Expectation(String description, Function<double[], Boolean> test) {}

    static final Map<String, Expectation> EXPECTED_MOTION = Map.of(
        "drive backward", new Expectation("move -x by > 0.5 m", d -> d[0] < -0.5),
        "drive forward", new Expectation("move +x by > 0.5 m", d -> d[0] > 0.5),
        "strafe left", new Expectation("move +y by > 0.5 m", d -> d[1] > 0.5),
        "strafe right", new Expectation("move -y by > 0.5 m", d -> d[1] < -0.5),
        "spin in place", new Expectation("turn > 90 deg, translate < 0.3 m", d -> Math.abs(d[2]) > Math.PI / 2 && Math.hypot(d[0], d[1]) < 0.3),
        "x-lock", new Expectation("translate < 0.05 m", d -> Math.hypot(d[0], d[1]) < 0.05),
        "drive and shoot (aim at hub)", new Expectation("translate < 0.1 m (shoot on the move is off)", d -> Math.hypot(d[0], d[1]) < 0.1)
    );

    static void checkSteps(List<Step> steps) {
        header(matchMode ? "SEGMENTS (estimated pose motion, and commanded vs. measured chassis speed while the command is steady)"
                         : "STEPS (true sim pose motion, and commanded vs. measured chassis speed)");
        System.out.printf("  %-32s %-6s %6s %7s %7s %8s   %-24s%n", "step", "mode", "len s", matchMode ? "dist m" : "dx m", matchMode ? "top m/s" : "dy m", "dθ deg", "steady-state speed err vx/vy/ω");
        double maxAccel = 0;
        String maxAccelStep = "";
        for (Step step : steps) {
            String mode = !step.enabled() ? "off" : step.auto() ? "auto" : "teleop";
            if (!step.enabled()) { // the sim resets the robot pose on disable, so motion here isn't meaningful
                System.out.printf("  %-32s %-6s %6.2f %7s %7s %8s   -%n", step.name(), mode, step.length(), "-", "-", "-");
                continue;
            }
            double[] p0 = struct(poseKey(), step.start() + 0.02), p1 = struct(poseKey(), step.end() - 0.001);
            double[] delta = p0 == null || p1 == null ? new double[] {0, 0, 0} : new double[] {p1[0] - p0[0], p1[1] - p0[1], 0};
            // total rotation, added up loop by loop (start vs. end heading alone can't tell 40 deg from 400 deg)
            double[] prev = null;
            double distance = 0, topSpeed = 0, prevT = 0;
            for (double t : loops(step.start() + 0.02, step.end())) {
                double[] p = struct(poseKey(), t);
                if (p == null) continue;
                if (prev != null) {
                    delta[2] += wrap(p[2] - prev[2]);
                    double d = Math.hypot(p[0] - prev[0], p[1] - prev[1]);
                    distance += d;
                    if (t > prevT) topSpeed = Math.max(topSpeed, d / (t - prevT));
                }
                prev = p;
                prevT = t;
            }
            if (matchMode) { // on a real robot, top speed from the wheels is less noisy than from the (vision-corrected) pose
                topSpeed = 0;
                for (double t : loops(step.start(), step.end())) {
                    double[] wheels = struct(SPEEDS_OUT, t);
                    if (wheels != null) topSpeed = Math.max(topSpeed, Math.hypot(wheels[0], wheels[1]));
                }
            }

            // actual acceleration, from the true sim pose (velocity over 5 loops, then change in that over 5 loops).
            // not from module states: those swing when the wheels steer, even though the robot itself can't change velocity that fast
            List<Double> stepLoops = loops(step.start(), step.end());
            for (int i = 10; i < stepLoops.size(); i++) {
                double[] va = trueVelocity(stepLoops.get(i - 10), stepLoops.get(i - 5)), vb = trueVelocity(stepLoops.get(i - 5), stepLoops.get(i));
                if (va == null || vb == null) continue;
                double accel = Math.hypot(vb[0] - va[0], vb[1] - va[1]) / ((stepLoops.get(i) - stepLoops.get(i - 10)) / 2);
                if (accel > maxAccel) {
                    maxAccel = accel;
                    maxAccelStep = step.name();
                }
            }

            // steady-state tracking error: scripted steps use their last 25% (the rest is the robot accelerating);
            // match segments use every loop where the command hasn't changed much in the last 0.5 s
            List<Double> ex = new ArrayList<>(), ey = new ArrayList<>(), ew = new ArrayList<>();
            {
                for (double t : loops(matchMode ? step.start() + 0.5 : step.end() - 0.25 * step.length(), step.end())) {
                    double[] cmd = struct(SPEEDS_IN, t), measured = struct(SPEEDS_OUT, t);
                    if (cmd == null || measured == null) continue;
                    if (matchMode) {
                        double[] earlier = struct(SPEEDS_IN, t - 0.5);
                        if (earlier == null || Math.hypot(cmd[0] - earlier[0], cmd[1] - earlier[1]) > 0.1 || Math.abs(cmd[2] - earlier[2]) > 0.2) continue;
                    }
                    double[] meas2 = measured;
                    ex.add(Math.abs(cmd[0] - meas2[0]));
                    ey.add(Math.abs(cmd[1] - meas2[1]));
                    ew.add(Math.abs(cmd[2] - meas2[2]));
                }
            }
            String tracking = ex.isEmpty() ? "-" : String.format("%.2f / %.2f / %.2f", mean(ex), mean(ey), mean(ew));
            System.out.printf("  %-32s %-6s %6.2f %7.2f %7.2f %8.1f   %s%n", step.name(), mode, step.length(),
                matchMode ? distance : delta[0], matchMode ? topSpeed : delta[1], Math.toDegrees(delta[2]), tracking);

            if (!ex.isEmpty() && !step.auto() && (mean(ex) > MAX_LINEAR_TRACKING_ERROR || mean(ey) > MAX_LINEAR_TRACKING_ERROR || mean(ew) > MAX_ANGULAR_TRACKING_ERROR)) {
                problem(String.format("'%s': measured chassis speed doesn't follow the command (err vx %.2f, vy %.2f m/s, ω %.2f rad/s)", step.name(), mean(ex), mean(ey), mean(ew)));
            }
            Expectation exp = EXPECTED_MOTION.get(step.name());
            if (exp != null && !exp.test().apply(delta)) {
                problem(String.format("'%s': expected robot to %s, but it moved dx %.2f, dy %.2f m, dθ %.0f deg", step.name(), exp.description(), delta[0], delta[1], Math.toDegrees(delta[2])));
            }
        }
        System.out.printf("  max %s linear acceleration: %.1f m/s^2 (during '%s')%n", matchMode ? "estimated" : "actual", maxAccel, maxAccelStep);
    }

    // where the robot is: the sim's ground truth if there is one, otherwise the robot's own best estimate
    static String poseKey() {
        return matchMode ? POSE_EST + "combinedPoseEstimate" : TRUE_POSE;
    }

    static double[] trueVelocity(double t0, double t1) {
        double[] a = struct(poseKey(), t0), b = struct(poseKey(), t1);
        if (a == null || b == null || t1 <= t0) return null;
        return new double[] {(b[0] - a[0]) / (t1 - t0), (b[1] - a[1]) / (t1 - t0)};
    }

    // how far the robot was from the auto's ORIGINAL path: drive/pathPose (the Choreo sample before any start blend), or before the
    // path starts (e.g. while driving to the start), drive/targetPose. also how long the auto took to start its path
    static String exportDir = null;

    static String cm(double meters) {
        return Double.isNaN(meters) ? "-" : String.format("%.0fcm", meters * 100);
    }

    static void checkAutoTracking(List<Step> steps) {
        header("AUTO PATH TRACKING (robot " + (matchMode ? "estimated" : "true") + " pose vs. the auto's original path)");
        System.out.printf("  %-40s %-14s %7s %7s %7s %7s %8s %7s %8s%n", "auto", "start method", "@0s", "@0.5s", "@1s", "@1.5s", "max >2s", "mean", "delay");
        boolean any = false;
        int skippedForFreezes = 0;
        StringBuilder series = new StringBuilder("auto,method,seconds,error_m%n".formatted());
        StringBuilder summary = new StringBuilder("auto,method,err_0s_m,err_0_5s_m,err_1s_m,err_1_5s_m,max_after_2s_m,path_start_delay_s%n".formatted());
        for (Step step : steps) {
            if (!step.auto()) continue;
            any = true;
            // the log only stores changes, so the same decision twice in a row is one sample; decisionCount goes up every time.
            // autos without a Choreo path (e.g. BackUpAndShoot) don't make a decision, so they show "-"
            double probe = step.start() + 0.3;
            boolean decidedNow = Math.max(lastSampleTime(OUT + "autoStart/decision", probe), lastSampleTime(OUT + "autoStart/decisionCount", probe)) >= step.start() - 0.1;
            Object decision = at(OUT + "autoStart/decision", probe);
            String method = decision == null || !decidedNow ? "-" : decision.toString().split(" ")[0];

            double e0 = Double.NaN, e05 = Double.NaN, e1 = Double.NaN, e15 = Double.NaN, maxAfter2 = 0, pathStart = Double.NaN;
            List<Double> all = new ArrayList<>();
            double lastFreeze = Double.NEGATIVE_INFINITY;
            for (double t : loops(step.start(), step.end())) {
                // (checked first, so a freeze right at the start of the auto still counts)
                // sim only: when the PC freezes, the sim's physics stops but the path's timer doesn't, so the path "runs away"
                // from the robot. that can't happen on a real robot, so skip the 2 s after any freeze
                if (!matchMode && num("/RealOutputs/LoggedRobot/FullCycleMS", t) > 100) lastFreeze = t;
                if (!matchMode && t - lastFreeze < 2.0) {
                    skippedForFreezes++;
                    continue;
                }
                // reference: the path's own pose while the path is running, otherwise wherever the auto is driving to
                boolean onPath = lastSampleTime(OUT + "drive/pathPose", t) >= Math.max(step.start(), t - 0.05);
                String refKey = onPath ? OUT + "drive/pathPose" : OUT + "drive/targetPose";
                if (lastSampleTime(refKey, t) < step.start()) continue; // nothing from this auto yet
                if (onPath && Double.isNaN(pathStart)) pathStart = t - step.start();


                double[] target = struct(refKey, t), pose = struct(poseKey(), t);
                if (target == null || pose == null) continue;
                double err = Math.hypot(target[0] - pose[0], target[1] - pose[1]);
                double since = t - step.start();
                series.append(String.format("\"%s\",%s,%.3f,%.4f%n", step.name(), method, since, err));
                all.add(err);
                if (Double.isNaN(e0)) e0 = err;
                // only count a sample for a checkpoint if it's close to it (samples right after a sim freeze are dropped)
                if (Double.isNaN(e05) && since >= 0.5 && since < 0.6) e05 = err;
                if (Double.isNaN(e1) && since >= 1.0 && since < 1.1) e1 = err;
                if (Double.isNaN(e15) && since >= 1.5 && since < 1.6) e15 = err;
                if (since >= 2.0) maxAfter2 = Math.max(maxAfter2, err);
            }
            System.out.printf("  %-40s %-14s %7s %7s %7s %7s %8s %7s %8s%n", step.name(), method,
                cm(e0), cm(e05), cm(e1), cm(e15), cm(maxAfter2), cm(mean(all)), Double.isNaN(pathStart) ? "-" : String.format("%.2fs", pathStart));
            summary.append(String.format("\"%s\",%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f%n", step.name(), method, e0, e05, e1, e15, maxAfter2, pathStart));
            if (maxAfter2 > 0.25) {
                problem(String.format("'%s': robot was still up to %.0f cm off the auto path more than 2 s in", step.name(), maxAfter2 * 100));
            }
        }
        if (!any) System.out.println("  no auto in this log");
        System.out.println("  (delay = time from auto start until the path itself started; driving to the start first costs this much auto time)");
        if (skippedForFreezes > 0) System.out.printf("  (skipped %d loops in the 2 s after PC freezes > 100 ms; sim-only, see comment in checkAutoTracking)%n", skippedForFreezes);

        if (exportDir != null && any) {
            try {
                File dir = new File(exportDir);
                dir.mkdirs();
                java.nio.file.Files.writeString(new File(dir, "auto_tracking.csv").toPath(), series.toString());
                java.nio.file.Files.writeString(new File(dir, "auto_summary.csv").toPath(), summary.toString());
                System.out.println("  wrote auto_tracking.csv and auto_summary.csv to " + dir.getPath());
            } catch (java.io.IOException e) {
                System.out.println("  could not write CSVs: " + e.getMessage());
            }
        }
    }

    static void checkModules(List<Step> steps) {
        header("SWERVE MODULES (setpoint vs. measured, teleop driving only)");
        List<List<Double>> speedErr = new ArrayList<>(), angleErr = new ArrayList<>(), steadyAngleErr = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            speedErr.add(new ArrayList<>());
            angleErr.add(new ArrayList<>());
            steadyAngleErr.add(new ArrayList<>());
        }
        for (Step step : steps) {
            if (!step.enabled() || step.auto()) continue;
            for (double t : loops(matchMode ? step.start() : step.mid(), step.end())) {
                double[] in = struct(MODULES_IN, t), out = struct(MODULES_OUT, t);
                if (in == null || out == null || in.length < 8 || out.length < 8) continue;
                for (int i = 0; i < 4; i++) {
                    double inSpeed = in[2 * i], inAngle = in[2 * i + 1], outSpeed = out[2 * i], outAngle = out[2 * i + 1];
                    if (Math.abs(inSpeed) < 0.1) continue; // angle doesn't matter when stopped
                    // a module pointed 180 deg off running backwards is the same thing
                    double angle = wrap(inAngle - outAngle);
                    double sign = 1;
                    if (Math.abs(angle) > Math.PI / 2) {
                        angle = wrap(angle + Math.PI);
                        sign = -1;
                    }
                    speedErr.get(i).add(Math.abs(inSpeed - sign * outSpeed));
                    angleErr.get(i).add(Math.toDegrees(Math.abs(angle)));
                    // steady: the setpoint angle hasn't moved more than 3 deg in the last 0.25 s (so the module had time to get there)
                    double[] earlier = struct(MODULES_IN, t - 0.25);
                    if (earlier != null && earlier.length >= 8 && Math.abs(earlier[2 * i]) > 0.1) {
                        double setpointMove = Math.abs(wrap(inAngle - earlier[2 * i + 1]));
                        setpointMove = Math.min(setpointMove, Math.PI - setpointMove); // a 180 deg flip of the setpoint is the same direction
                        if (Math.toDegrees(setpointMove) < 3) steadyAngleErr.get(i).add(Math.toDegrees(Math.abs(angle)));
                    }
                }
            }
        }
        for (int i = 0; i < 4; i++) {
            System.out.printf("  module %d: speed err mean %.2f m/s, angle err mean %.1f deg (p95 %.1f), when steady %.1f deg (p95 %.1f)  [%d samples, %d steady]%n",
                i, mean(speedErr.get(i)), mean(angleErr.get(i)), percentile(angleErr.get(i), 0.95),
                mean(steadyAngleErr.get(i)), percentile(steadyAngleErr.get(i), 0.95), angleErr.get(i).size(), steadyAngleErr.get(i).size());
            // judged on steady driving only: right after a quick direction change every module is briefly behind, which is normal
            if (!steadyAngleErr.get(i).isEmpty() && mean(steadyAngleErr.get(i)) > MAX_MODULE_ANGLE_ERROR_DEG) {
                problem(String.format("module %d steering doesn't follow its setpoint even when steady (mean %.1f deg off)", i, mean(steadyAngleErr.get(i))));
            }
        }
    }

    static void checkPoseEstimation(List<Step> steps) {
        header("POSE ESTIMATION (vs. true sim pose, while enabled)");
        Map<String, List<Double>> transErr = new LinkedHashMap<>(), headErr = new LinkedHashMap<>();
        for (String est : List.of("odometry", "vision", "combined")) {
            transErr.put(est, new ArrayList<>());
            headErr.put(est, new ArrayList<>());
        }
        int jumps = 0;
        double worstJump = 0, worstJumpTime = 0;
        double[] prevCombined = null, prevTrue = null;

        for (Step step : steps) {
            if (!step.enabled()) {
                prevCombined = null;
                continue;
            }
            for (double t : loops(step.start() + TRANSITION_IGNORE, step.end())) {
                double[] truth = struct(TRUE_POSE, t);
                if (truth == null) continue;
                for (String est : transErr.keySet()) {
                    double[] p = struct(POSE_EST + est + "PoseEstimate", t);
                    if (p == null) continue;
                    transErr.get(est).add(Math.hypot(p[0] - truth[0], p[1] - truth[1]));
                    headErr.get(est).add(Math.toDegrees(Math.abs(wrap(p[2] - truth[2]))));
                }

                double[] combined = struct(POSE_EST + "combinedPoseEstimate", t);
                if (prevCombined != null && combined != null) {
                    double jump = Math.hypot(combined[0] - prevCombined[0], combined[1] - prevCombined[1]);
                    double trueMove = Math.hypot(truth[0] - prevTrue[0], truth[1] - prevTrue[1]);
                    if (jump - trueMove > POSE_JUMP) {
                        jumps++;
                        if (jump > worstJump) {
                            worstJump = jump;
                            worstJumpTime = t - logStart;
                        }
                    }
                }
                prevCombined = combined;
                prevTrue = truth;
            }
        }

        for (String est : transErr.keySet()) {
            System.out.printf("  %-9s position err mean %.3f m, max %.3f m   heading err mean %.2f deg, max %.2f deg%n",
                est, mean(transErr.get(est)), max(transErr.get(est)), mean(headErr.get(est)), max(headErr.get(est)));
        }
        System.out.println("  (vision-only estimate ignores odometry entirely, so it's expected to be the noisiest)");

        if (mean(transErr.get("combined")) > MAX_COMBINED_POSE_ERROR) {
            problem(String.format("combined pose estimate is off by %.2f m on average", mean(transErr.get("combined"))));
        }
        if (max(transErr.get("odometry")) > MAX_ODOMETRY_POSE_ERROR) {
            problem(String.format("odometry drifted up to %.2f m from the true pose", max(transErr.get("odometry"))));
        }
        if (mean(headErr.get("combined")) > MAX_HEADING_ERROR_DEG) {
            problem(String.format("combined heading is off by %.1f deg on average", mean(headErr.get("combined"))));
        }
        if (jumps > 0) {
            problem(String.format("combined pose jumped > %.2f m in one loop %d time(s) (worst %.2f m at t=%.1f s)", POSE_JUMP, jumps, worstJump, worstJumpTime));
        }
    }

    // real robot: there's no true pose, so compare odometry (wheels + gyro) to the combined (vision-corrected) estimate.
    // they drift apart from wheel slip and collisions, and the combined estimate jumps when vision corrects it
    static void checkPoseEstimationMatch(List<Step> steps) {
        header("POSE ESTIMATION (odometry vs. vision-corrected estimate, while enabled)");
        List<Double> drift = new ArrayList<>(), headingDrift = new ArrayList<>();
        int snaps = 0;
        double worstSnap = 0, worstSnapTime = 0;
        for (Step step : steps) {
            if (!step.enabled()) continue;
            double[] prevCombined = null, prevOdometry = null;
            for (double t : loops(step.start() + TRANSITION_IGNORE, step.end())) {
                double[] combined = struct(POSE_EST + "combinedPoseEstimate", t), odometry = struct(POSE_EST + "odometryPoseEstimate", t);
                if (combined == null || odometry == null) continue;
                drift.add(Math.hypot(combined[0] - odometry[0], combined[1] - odometry[1]));
                headingDrift.add(Math.toDegrees(Math.abs(wrap(combined[2] - odometry[2]))));
                if (prevCombined != null) {
                    double jump = Math.hypot(combined[0] - prevCombined[0], combined[1] - prevCombined[1])
                        - Math.hypot(odometry[0] - prevOdometry[0], odometry[1] - prevOdometry[1]);
                    if (jump > POSE_JUMP) {
                        snaps++;
                        if (jump > worstSnap) {
                            worstSnap = jump;
                            worstSnapTime = t;
                        }
                    }
                }
                prevCombined = combined;
                prevOdometry = odometry;
            }
        }
        System.out.printf("  odometry vs. combined: mean %.2f m, max %.2f m apart; heading mean %.1f deg, max %.1f deg apart%n",
            mean(drift), max(drift), mean(headingDrift), max(headingDrift));
        System.out.printf("  vision snapped the pose > %.2f m in one loop %d time(s)%s%n", POSE_JUMP, snaps,
            snaps == 0 ? "" : String.format(" (worst %.2f m at t=%.1f s, during '%s')", worstSnap, worstSnapTime - logStart, stepAt(steps, worstSnapTime)));
        System.out.println("  (some drift is normal: odometry never gets vision corrections. a sudden big snap usually means one bad vision measurement got through)");
        if (worstSnap > 1.0) {
            problem(String.format("pose estimate jumped %.2f m in one loop at t=%.1f s (likely a bad vision measurement; check for tag ambiguity or a camera transform error)", worstSnap, worstSnapTime - logStart));
        }
        if (max(headingDrift) > 15) {
            problem(String.format("odometry heading and the vision-corrected heading disagree by up to %.0f deg (gyro drift or a camera's rotation is wrong)", max(headingDrift)));
        }
    }

    // brownouts, CAN, comms, temperatures and stalled/jammed mechanisms
    static void checkRobotHealth(List<Step> steps) {
        header("ROBOT HEALTH (brownouts, CAN, comms, temperatures, stalls)");

        // brownouts
        int brownouts = 0;
        boolean wasBrownedOut = false;
        for (Sample s : series("/SystemStats/BrownedOut")) {
            boolean browned = Boolean.TRUE.equals(s.value());
            if (browned && !wasBrownedOut) {
                brownouts++;
                problem(String.format("BROWNOUT at t=%.1f s during '%s' (battery %.2f V): motors cut out", s.t() - logStart, stepAt(steps, s.t()),
                    num("/SystemStats/BatteryVoltage", s.t())));
            }
            wasBrownedOut = browned;
        }
        System.out.println("  brownouts: " + brownouts);

        // CAN bus
        List<Double> utilization = new ArrayList<>();
        for (Sample s : series("/SystemStats/CANBus/Utilization")) {
            if (bool("/DriverStation/Enabled", s.t())) utilization.add((Double) s.value() * 100);
        }
        double offCount = growth("/SystemStats/CANBus/OffCount"), txFull = growth("/SystemStats/CANBus/TxFullCount");
        double rxErrors = maxOf("/SystemStats/CANBus/ReceiveErrorCount"), txErrors = maxOf("/SystemStats/CANBus/TransmitErrorCount");
        System.out.printf("  CAN: utilization %s; bus-off events %.0f, TX buffer full %.0f, max RX/TX error counts %.0f/%.0f%n",
            utilization.isEmpty() ? "not logged" : String.format("mean %.0f%%, max %.0f%% (enabled)", mean(utilization), max(utilization)),
            offCount, txFull, rxErrors, txErrors);
        if (offCount > 0) problem(String.format("CAN bus went off %.0f time(s) (wiring problem: check connectors and termination)", offCount));
        if (!utilization.isEmpty() && mean(utilization) > 80) problem(String.format("CAN bus is very busy (%.0f%% average); lower status signal rates", mean(utilization)));
        if (rxErrors > 100 || txErrors > 100) problem(String.format("CAN error counts got high (RX %.0f, TX %.0f): likely a loose or damaged CAN connection", rxErrors, txErrors));

        // comms with the driver station
        double commsDisables = growth("/SystemStats/CommsDisableCount");
        int dsDrops = 0;
        boolean seenAttached = false, attached = false;
        for (Sample s : series("/DriverStation/DSAttached")) {
            boolean a = Boolean.TRUE.equals(s.value());
            if (seenAttached && attached && !a) dsDrops++;
            if (a) seenAttached = true;
            attached = a;
        }
        System.out.printf("  comms: %.0f comms-loss disable(s), driver station disconnected %d time(s)%n", commsDisables, dsDrops);
        if (commsDisables > 0 || dsDrops > 0) {
            problem(String.format("lost comms with the driver station (%.0f disable(s), %d disconnect(s)): check the radio, its power and the ethernet cable", commsDisables, dsDrops));
        }

        // motor temperatures
        StringBuilder temps = new StringBuilder();
        for (String key : types.keySet()) {
            if (!key.startsWith("/subsystems/") || !key.endsWith("/Temperature")) continue;
            double maxTemp = maxOf(key);
            String name = key.replace("/subsystems/", "").replace("/Temperature", "");
            if (maxTemp > 150) { // no motor survives this; it's a bad reading (e.g. the controller isn't the type the code expects)
                temps.append(String.format("%s INVALID (%.0fC)  ", name, maxTemp));
                System.out.printf("  [warning] %s temperature reads %.0f C, which is impossible: the motor controller isn't reporting real data (wrong controller type in code, or not connected?)%n", name, maxTemp);
                continue;
            }
            temps.append(String.format("%s %.0fC  ", name, maxTemp));
            if (maxTemp > 80) problem(String.format("%s motor reached %.0f C (it will start limiting power; let it cool between matches)", name, maxTemp));
        }
        System.out.println("  max motor temps: " + (temps.length() == 0 ? "not logged" : temps.toString().trim()));

        // stalls: lots of current but not moving, for a while. on a mechanism that's usually a jam; on the drive, usually pushing
        record Motor(String name, String current, String velocity, double minCurrent, double maxVelocity, boolean isDrive) {}
        List<Motor> motors = new ArrayList<>();
        for (int i = 0; i < 4; i++) {
            String base = "/subsystems/drive/module" + i + "/";
            motors.add(new Motor("drive " + i, base + "DriveCurrentAmps", base + "DriveVelocityRadPerSec", 50, 2, true));
        }
        motors.add(new Motor("shooter", SHOOTER + "Current", SHOOTER + "Velocity", 30, 1, false));
        motors.add(new Motor("kicker", KICKER + "Current", KICKER + "Velocity", 30, 1, false));
        motors.add(new Motor("intake", INTAKE + "Current", INTAKE + "Velocity", 30, 0.5, false));
        motors.add(new Motor("pivot", PIVOT + "Current", PIVOT + "Velocity", 30, 0.02, false));
        int stalls = 0;
        for (Motor m : motors) {
            double stallStart = -1;
            for (Step step : steps) {
                if (!step.enabled()) continue;
                for (double t : loops(step.start(), step.end())) {
                    boolean stalled = Math.abs(num(m.current(), t)) > m.minCurrent() && Math.abs(num(m.velocity(), t)) < m.maxVelocity();
                    if (stalled && stallStart < 0) stallStart = t;
                    if (!stalled && stallStart >= 0) {
                        if (t - stallStart > 0.5) {
                            stalls++;
                            String msg = String.format("%s stalled for %.1f s at t=%.1f s during '%s' (> %.0f A but not moving)", m.name(), t - stallStart, stallStart - logStart, step.name(), m.minCurrent());
                            if (m.isDrive()) System.out.println("  [warning] " + msg + ", probably pushing against something");
                            else problem(msg + ": jammed?");
                        }
                        stallStart = -1;
                    }
                }
                stallStart = -1;
            }
        }
        if (stalls == 0) System.out.println("  stalls: none");
    }

    // how much a counter went up over the log
    static double growth(String key) {
        List<Sample> s = series(key);
        if (s.isEmpty() || !(s.get(0).value() instanceof Double)) return 0;
        return (Double) s.get(s.size() - 1).value() - (Double) s.get(0).value();
    }

    static double maxOf(String key) {
        return series(key).stream().filter(s -> s.value() instanceof Double).mapToDouble(s -> (Double) s.value()).max().orElse(Double.NaN);
    }

    static void checkVision(List<Step> steps) {
        header("VISION");
        for (String cam : List.of("camera0", "camera1")) {
            int accepted = 0, rejected = 0, framesWithTags = 0, loops = 0;
            for (Step step : steps) {
                if (!step.enabled()) continue;
                for (double t : loops(step.start(), step.end())) {
                    loops++;
                    accepted += pose3dCount(VISION + cam + "/robotPosesAccepted", t);
                    rejected += pose3dCount(VISION + cam + "/robotPosesRejected", t);
                    if (pose3dCount(VISION + cam + "/tagPoses", t) > 0) framesWithTags++;
                }
            }
            boolean connected = bool("/subsystems/poseEstimator/vision/" + cam + "/Connected", logEnd);
            System.out.printf("  %s: connected %s, sees tags in %d/%d enabled loops, pose observations accepted %d, rejected %d%n",
                cam, connected, framesWithTags, loops, accepted, rejected);
            if (!connected) problem(cam + " is not connected");
            if (accepted + rejected > 0 && rejected > accepted) {
                problem(String.format("%s rejects more pose observations than it accepts (%d vs %d)", cam, rejected, accepted));
            }
        }
    }

    static void checkShooter(List<Step> steps) {
        header("SHOOTER AND KICKER");
        // find periods where the shooter is commanded to spin
        List<double[]> periods = new ArrayList<>(); // start, end
        double periodStart = -1;
        for (double t : loops(logStart, logEnd + 1)) {
            boolean spinning = num(SHOOTER + "VelocitySetpoint", t) > 1;
            if (spinning && periodStart < 0) periodStart = t;
            if (!spinning && periodStart >= 0) {
                periods.add(new double[] {periodStart, t});
                periodStart = -1;
            }
        }
        if (periodStart >= 0) periods.add(new double[] {periodStart, logEnd});

        if (periods.isEmpty()) {
            System.out.println("  shooter was never commanded to spin");
        }
        for (double[] period : periods) {
            double reached = -1;
            List<Double> err = new ArrayList<>();
            double maxCurrent = 0;
            for (double t : loops(period[0], period[1])) {
                double sp = num(SHOOTER + "VelocitySetpoint", t), v = num(SHOOTER + "Velocity", t);
                maxCurrent = Math.max(maxCurrent, Math.abs(num(SHOOTER + "Current", t)));
                if (reached < 0 && Math.abs(sp - v) < 5) reached = t;
                if (reached >= 0) err.add(Math.abs(sp - v));
            }
            String stepName = stepAt(steps, period[0]);
            double spinUp = reached < 0 ? Double.NaN : reached - period[0];
            System.out.printf("  shooter spun %.1f s during '%s': setpoint ~%.1f rps, reached within 5 rps after %s, then mean err %.2f rps, max current %.0f A%n",
                period[1] - period[0], stepName, num(SHOOTER + "VelocitySetpoint", (period[0] + period[1]) / 2),
                reached < 0 ? "NEVER" : String.format("%.2f s", spinUp), mean(err), maxCurrent);
            if (period[1] - period[0] > SHOOTER_SPINUP_LIMIT && (reached < 0 || spinUp > SHOOTER_SPINUP_LIMIT)) {
                problem(String.format("shooter took %s to reach speed during '%s'", reached < 0 ? "forever" : String.format("%.2f s", spinUp), stepName));
            }
        }

        // kicker: compare velocity to setpoint over the last 30% of each constant-setpoint segment
        List<Sample> kickerSp = series(KICKER + "VelocitySetpoint");
        for (int i = 0; i < kickerSp.size(); i++) {
            double start = kickerSp.get(i).t(), end = i + 1 < kickerSp.size() ? kickerSp.get(i + 1).t() : logEnd;
            double sp = (Double) kickerSp.get(i).value();
            double length = matchMode ? end - start : loops(start, end).size() * 0.02; // sim: loops, not clock time (PC freezes stretch the clock)
            if (Math.abs(sp) < 1 || length < 0.3) continue;
            List<Double> err = new ArrayList<>();
            for (double t : loops(end - 0.3 * (end - start), end)) err.add(Math.abs(sp - num(KICKER + "Velocity", t)));
            System.out.printf("  kicker at %.0f for %.2f s during '%s': steady-state err %.1f%n", sp, end - start, stepAt(steps, start), mean(err));
            if (mean(err) > 0.15 * Math.abs(sp) + 2) {
                problem(String.format("kicker doesn't reach its %.0f setpoint during '%s' (err %.1f)", sp, stepAt(steps, start), mean(err)));
            }
        }
    }

    static void checkPivotAndIntake(List<Step> steps) {
        header("PIVOT AND INTAKE");
        List<Sample> sp = series(PIVOT + "PositionSetpoint");
        for (int i = 0; i < sp.size(); i++) {
            double start = sp.get(i).t(), end = i + 1 < sp.size() ? sp.get(i + 1).t() : logEnd;
            double target = (Double) sp.get(i).value();
            if (end - start < 0.1 || (i == 0 && target == 0)) continue; // the default value logged before the subsystem first sets its setpoint
            // in sim, count loops instead of clock time: a PC hiccup freezes the sim's physics but not the clock
            double settled = -1;
            int loopCount = 0;
            for (double t : loops(start, end)) {
                if (Math.abs(num(PIVOT + "Position", t) - target) < PIVOT_TOLERANCE) {
                    settled = matchMode ? t : start + loopCount * 0.02;
                    break;
                }
                loopCount++;
            }
            double finalErr = Math.abs(num(PIVOT + "Position", end - 0.001) - target);
            System.out.printf("  pivot -> %.3f rot during '%s': settled after %s, final err %.3f rot%n",
                target, stepAt(steps, start), settled < 0 ? "NEVER" : String.format("%.2f s", settled - start), finalErr);
            if (end - start > PIVOT_SETTLE_LIMIT && (settled < 0 || settled - start > PIVOT_SETTLE_LIMIT)) {
                problem(String.format("pivot didn't reach %.3f rot within %.1f s during '%s'", target, PIVOT_SETTLE_LIMIT, stepAt(steps, start)));
            }
        }

        for (Step step : steps) {
            if (!step.name().contains("intake") || !step.enabled()) continue;
            double maxVolts = 0;
            for (double t : loops(step.start(), step.end())) maxVolts = Math.max(maxVolts, Math.abs(num(INTAKE + "Voltage", t)));
            System.out.printf("  intake during '%s': max %.1f V%n", step.name(), maxVolts);
            if (maxVolts < 1) {
                problem("intake never ran during '" + step.name() + "' (it only runs when the pivot is down)");
            }
        }
    }

    static void checkCurrentsAndBattery(List<Step> steps) {
        header("MOTOR CURRENTS (max while enabled) AND BATTERY");
        Map<String, String> motors = new LinkedHashMap<>();
        for (int i = 0; i < 4; i++) {
            motors.put("drive " + i, "/subsystems/drive/module" + i + "/DriveCurrentAmps");
            motors.put("turn " + i, "/subsystems/drive/module" + i + "/TurnCurrentAmps");
        }
        motors.put("shooter", SHOOTER + "Current");
        motors.put("kicker", KICKER + "Current");
        motors.put("pivot", PIVOT + "Current");
        motors.put("intake", INTAKE + "Current");

        StringBuilder line = new StringBuilder("  ");
        int col = 0;
        for (var motor : motors.entrySet()) {
            double maxAmps = 0;
            for (Sample s : series(motor.getValue())) {
                if (bool("/DriverStation/Enabled", s.t())) maxAmps = Math.max(maxAmps, Math.abs((Double) s.value()));
            }
            line.append(String.format("%-8s %5.0f A    ", motor.getKey(), maxAmps));
            if (++col % 4 == 0) line.append("\n  ");
        }
        System.out.println(line.toString().stripTrailing());

        double minVolts = Double.MAX_VALUE, maxVolts = 0, minTime = 0;
        for (Sample s : series("/SystemStats/BatteryVoltage")) {
            if (stepAt(steps, s.t()).equals("?")) continue; // before the robot program finished starting
            double v = (Double) s.value();
            maxVolts = Math.max(maxVolts, v);
            if (v < minVolts) {
                minVolts = v;
                minTime = s.t();
            }
        }
        if (maxVolts - minVolts < 0.05) {
            System.out.printf("  battery: constant %.2f V (battery sag isn't modeled in this sim, so this can't be checked)%n", minVolts);
        } else {
            System.out.printf("  battery: min %.2f V during '%s'%n", minVolts, stepAt(steps, minTime));
            if (minVolts < MIN_BATTERY_VOLTAGE) {
                problem(String.format("battery sagged to %.2f V during '%s'", minVolts, stepAt(steps, minTime)));
            }
        }
    }

    // things that are wrong with the simulation itself rather than the robot code; reported but not counted as problems
    static void checkSimFidelity() {
        header("SIM FIDELITY NOTES (sim-only, not counted as robot problems)");
        Map<String, String> mechanisms = new LinkedHashMap<>();
        mechanisms.put("shooter", SHOOTER);
        mechanisms.put("kicker", KICKER);
        mechanisms.put("pivot", PIVOT);
        mechanisms.put("intake", INTAKE);
        boolean any = false;
        for (var m : mechanisms.entrySet()) {
            double maxVolts = series(m.getValue() + "Voltage").stream().mapToDouble(s -> Math.abs((Double) s.value())).max().orElse(0);
            double maxAmps = series(m.getValue() + "Current").stream().mapToDouble(s -> Math.abs((Double) s.value())).max().orElse(0);
            if (maxVolts > 13) {
                System.out.printf("  %s sim applied up to %.0f V (a real motor can't exceed battery voltage, ~12 V), so it responds faster than the real one%n", m.getKey(), maxVolts);
                any = true;
            }
            if (maxAmps > 150) {
                System.out.printf("  %s sim drew up to %.0f A (no current limit modeled; the real motor's limit would slow it down)%n", m.getKey(), maxAmps);
                any = true;
            }
        }
        if (!any) System.out.println("  none");
    }

    static String stepAt(List<Step> steps, double t) {
        for (Step s : steps) {
            if (t >= s.start() && t < s.end()) return s.name();
        }
        return "?";
    }

    static File newestLog() {
        File[] logs = new File("logs").listFiles((dir, name) -> name.endsWith(".wpilog"));
        if (logs == null || logs.length == 0) {
            throw new IllegalStateException("no .wpilog files in logs/ (run ./gradlew simulateJava -PscriptedSim first)");
        }
        return Arrays.stream(logs).max(Comparator.comparingLong(File::lastModified)).get();
    }
}
