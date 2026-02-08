package frc.robot.subsystems.poseEstimator.vision;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.dyn4j.geometry.Rotation;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.PoseObservation;
import frc.robot.subsystems.poseEstimator.vision.CameraIO.VisionIOInputs;


public class CameraIOObjectDetection implements DetectionIO  {
    

    private List<PhotonTrackedTarget> targetList = new LinkedList<>();
        protected final PhotonCamera camera;
        protected final Transform3d robotToCamera;
        List<ObjectDetection> poseObservations = new LinkedList<>();
        /**
         * Creates a new VisionIOObjectDetection.
         *
         * @param name The configured name of the camera.
         * @param rotationSupplier The 3D position of the camera relative to the robot.
         */
        public  CameraIOObjectDetection(String name, Transform3d robotToCamera) {
            camera = new PhotonCamera(name);
            this.robotToCamera = robotToCamera;
        }





        

        /**
         * loops through the pipeline results and adds target data results to a list of targetData
         */
        @Override
        public void updateInputs(DetectionIOInputs inputs) {
            inputs.connected = camera.isConnected();
            for (var result : camera.getAllUnreadResults()) {
                if (result.hasTargets()) {
                poseObservations.add(new ObjectDetection(
                    result.getTimestampSeconds(),
                    targetToPose(result.getBestTarget()),
                    result.getBestTarget().getDetectedObjectClassID(),
                    PoseObservationType.PHOTONVISION));
                   
        
                }
            }
            }
        

        /**
         * sort and find closest target
         */
        public PhotonTrackedTarget getClosestObject() {
            PhotonTrackedTarget closestTarget = new PhotonTrackedTarget();

            if (targetList.size() > 0) {
                double closestArea = 0;
                        
                for (PhotonTrackedTarget target : targetList) {
                    double area = target.getArea();

                    //closer the target, the larger the area
                    if (area > closestArea) {
                        closestTarget = target;
                        closestArea = area;
                    }
                }
            }
            return closestTarget;
        }


        public Pose2d targetToPose(PhotonTrackedTarget fuel) {
            Transform2d fuelTransform = new Transform2d(
                fuel.getBestCameraToTarget().getMeasureX(), fuel.getBestCameraToTarget().getMeasureY(), fuel.getBestCameraToTarget().getRotation().toRotation2d()
            );
            
            Transform2d robotToCamera2d = new Transform2d(robotToCamera.getTranslation().toTranslation2d(), 
                                  robotToCamera.getRotation().toRotation2d());


            Transform2d fuelPose =  fuelTransform.plus(robotToCamera2d.inverse());

            return new Pose2d().transformBy(fuelPose);
        }

}
