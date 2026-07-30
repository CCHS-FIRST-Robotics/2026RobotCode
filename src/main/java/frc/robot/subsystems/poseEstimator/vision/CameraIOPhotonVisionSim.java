/**
 * Based on https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-TalonSwerveTemplate-MapleSim-Enhanced/blob/main/src/main/java/frc/robot/subsystems/vision/VisionIOPhotonVisionSim.java
 */

package frc.robot.subsystems.poseEstimator.vision;

import static frc.robot.subsystems.poseEstimator.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class CameraIOPhotonVisionSim extends CameraIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name The name of the camera.
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public CameraIOPhotonVisionSim(
        String name, 
        Transform3d robotToCameraTransform, 
        SimCameraProperties cameraProperties, 
        Supplier<Pose2d> poseSupplier
    ) {
        super(name, robotToCameraTransform);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagLayout);
        }

        // Add sim camera
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, robotToCameraTransform);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}