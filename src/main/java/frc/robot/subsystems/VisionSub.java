package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.Collection;
import java.util.Collections;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSub extends SubsystemBase {

    private final PhotonCamera camera;
    public PhotonTrackedTarget currentTarget;

    public Pigeon2 pigeon2 = new Pigeon2(14); 
    private PhotonPoseEstimator photonPoseEstimator; 
    PhotonPipelineResult result = new PhotonPipelineResult(); 



    private final CommandSwerveDrivetrain m_drivetrain; 
    PhotonPoseEstimator photonEstimator; 
    

    public VisionSub(CommandSwerveDrivetrain drivetrain, String cameraName) {
        camera = new PhotonCamera(cameraName);
        m_drivetrain = drivetrain; 


    }

    @Override
    public void periodic() {


        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (!results.isEmpty()) {
            PhotonPipelineResult latest = results.get(results.size() - 1);

            if (latest.hasTargets()) {
                currentTarget = latest.getBestTarget();
            } else {
                currentTarget = null;
            }
        }

    }


    public boolean hasTarget() {
        return currentTarget != null;
    }

    //get the fore and aft distance to the target
    public double getTargetX() {
        if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
            return currentTarget.bestCameraToTarget.getX();
        }
        return 0.0;
    }

    //get the left and right distance to cam center
    public double getTargetY() {
      if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
        return currentTarget.bestCameraToTarget.getY();
      }
      return 0.0;
    }

    public double getYaw() {
      if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
        var result = camera.getLatestResult(); 
        double targetYaw = result.getBestTarget().getYaw(); 
        return targetYaw; 
      }
      return 0.0; 
    }

    public double getHubY() {  //todo: get a better id identification system, all the values present on the blue hub below.
        // if ((Math.abs(currentTarget.bestCameraToTarget.getY()) <= 0.1)) {
        //     return 0.0; 
        // }
        // else {
        //     return currentTarget.bestCameraToTarget.getY(); 
        // }
        double targetYaw = 0.0; 
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()){
            var result = results.get(results.size() -1); 
            if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                    targetYaw = target.getYaw(); 
                    if (target.getFiducialId() == 25 || target.getFiducialId() == 18) {
                        if (target.getYaw() <= 0.2) {
                            return 0.0; 
                        }
                    }
                }
            }
        }
        return currentTarget.bestCameraToTarget.getY(); 


        // if (currentTarget != null && currentTarget.bestCameraToTarget != null && (currentTarget.getFiducialId()==18||currentTarget.getFiducialId()==19||currentTarget.getFiducialId()==20||currentTarget.getFiducialId()==22||currentTarget.getFiducialId()==25)) {
        //     return currentTarget.bestCameraToTarget.getY();

        // }
    }

    public Optional<PhotonTrackedTarget> getTarget() {
        return Optional.ofNullable(currentTarget);
    }
}
