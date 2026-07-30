/**
 * Based on https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/main/src/main/java/frc/robot/subsystems/vision/VisionConstants.java
 */

package frc.robot.subsystems.poseEstimator.vision;

import static edu.wpi.first.units.Units.*;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String[] cameraNames = {"FrontLeft", "FrontRight"};

    // Robot to camera transforms
    public static Transform3d[] robotToCameraTransforms = {
        new Transform3d(
            Inches.of(-9.8), 
            Inches.of(-10.75), 
            Inches.of(8.25), 
            new Rotation3d(Degrees.of(0), Degrees.of(-30), Degrees.of(200))
        ), 
        new Transform3d(
            Inches.of(-10.75), 
            Inches.of(10), 
            Inches.of(19),
            new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(0))
        )
    };

    // Camera properties (get these from photonvisions)
    public static SimCameraProperties[] cameraProperties = {
        new SimCameraProperties()
        .setCalibration(1920, 1400, Rotation2d.fromDegrees(92))
        .setCalibError(0.25, 0.08)
        .setFPS(30), 
        new SimCameraProperties()
        .setCalibration(1920, 1400, Rotation2d.fromDegrees(92))
        .setCalibError(0.25, 0.08)
        .setFPS(30), 
    };

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
