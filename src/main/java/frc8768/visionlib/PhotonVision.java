package frc8768.visionlib;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.Collections;
import java.util.List;

public class PhotonVision extends Vision {
    private final PhotonCamera camera;

    /**
     * @param type One of {@link Type}
     *             NOTE: Limelight 3 is pi for now.
     */
    public PhotonVision(Type type) {
        camera = new PhotonCamera(type.name);
    }

    public List<Object> getTargets() {
        if(camera.getLatestResult().hasTargets()) {
            return Collections.singletonList(camera.getLatestResult().targets);
        }
        return null;
    }

    public void changePipeline(int index) {
        camera.setPipelineIndex(index);
    }

    public PhotonTrackedTarget getBestTarget() {
        if(camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget();
        }
        return null;
    }

    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        PhotonTrackedTarget target = getBestTarget();
        if(target != null) {
            double angleToGoalDegrees = mountAngle + (topY ? getMaxPointY() : target.getBestCameraToTarget().getY());
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            double distance = (goalHeight - mountHeight) / Math.tan(angleToGoalRadians);
            if(distance <= 0) {
                return -1;
            }
            return distance;
        }
        return -1;
    }

    public double getMaxPointY() {
        PhotonTrackedTarget target = getBestTarget();
        if(target != null) {
            double maxY = 0;
            for(TargetCorner corner : target.getDetectedCorners()) {
                if(corner.y > maxY) {
                    maxY = corner.y;
                }
            }
        }
        return -1;
    }

    @Override
    public int getTargetID() {
        PhotonTrackedTarget target = getBestTarget();
        if(target != null) {
            return target.getFiducialId();
        }
        return -1;
    }

    public enum Type {
        /**
         * Raspberry Pi
         */
        PI("pi"),
        /**
         * Limelight 1 and 2
         */
        LIMELIGHT("limelight"),
        /**
         * Glowworm
         */
        GLOWWORM("glowworm");

        private String name;
        Type(String name) {
            this.name = name;
        }
    }
}
