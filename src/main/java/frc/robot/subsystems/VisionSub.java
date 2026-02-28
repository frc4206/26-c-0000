package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSub extends SubsystemBase {

    private final PhotonCamera camera;

    private PhotonTrackedTarget currentTarget;

    public VisionSub(String cameraName) {
        camera = new PhotonCamera(cameraName);
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

    public double getTargetX() {
        if (currentTarget != null && currentTarget.bestCameraToTarget != null) {
            return currentTarget.bestCameraToTarget.getX();
        }
        return 0.0;
    }

    public Optional<PhotonTrackedTarget> getTarget() {
        return Optional.ofNullable(currentTarget);
    }
}
