package frc8768.visionlib;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

// API for getting Vision
public class Vision {
    private PhotonCamera camera;

    // Name can be:
    // limelight
    // glowworm
    // pi
    public Vision(String name) {
        camera = new PhotonCamera(name);
    }

    public List<PhotonTrackedTarget> getTargets() {
        if(camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().targets;
        }
        return null;
    }

    public PhotonTrackedTarget getBestTarget() {
        if(camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget();
        }
        return null;
    }
}
