package frc8768.visionlib;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.List;

/**
 * API for Vision-related things.
 */
public class Vision {
    private final PhotonCamera camera;

    /**
     * @param name pi, limelight, or glowworm.
     *             NOTE: Limelight 3 is pi for now.
     */
    public Vision(String name) {
        camera = new PhotonCamera(name);
    }

    /**
     * Get all targets.
     *
     * @return All targets in view.
     */
    public List<PhotonTrackedTarget> getTargets() {
        if(camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().targets;
        }
        return null;
    }

    /**
     * Change the current pipeline.
     *
     * @param index Pipeline index, depends on your configuration.
     */
    public void changePipeline(int index) {
        camera.setPipelineIndex(index);
    }

    /**
     * Get the best target.
     *
     * @return Best target, null if none
     */
    public PhotonTrackedTarget getBestTarget() {
        if(camera.getLatestResult().hasTargets()) {
            return camera.getLatestResult().getBestTarget();
        }
        return null;
    }

    /**
     * Get the Distance to target via trigonometry.
     *
     * @param mountAngle The amount of degrees back the limelight is.
     * @param mountHeight Height in inches from center of limelight lens to floor.
     * @param goalHeight Distance in inches from floor to center of target.
     * @return Distance to target, returns -1 on fail.
     */
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        PhotonTrackedTarget target = getBestTarget();
        if(target != null) {
            double angleToGoalDegrees = mountAngle + (topY ? getMaxPointY() : target.getBestCameraToTarget().getY());
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            double distance = (goalHeight - mountHeight) / Math.tan(angleToGoalRadians);
            if(distance <= 0) {
                return -1;
            }
            return distance;
        }
        return -1;
    }

    /**
     * Sometimes, you may want to find the maximum Y for corners for the
     * intake to suck in the top of a cone, or cube. This function is helpful for that.
     *
     * @return The maximum Y point for all corners
     */
    public double getMaxPointY() {
        PhotonTrackedTarget target = getBestTarget();
        if(target != null) {
            double maxY = 0;
            for(TargetCorner corner : target.getDetectedCorners()) {
                if(corner.y > maxY) {
                    maxY = corner.y;
                }
            }
            if(maxY == 0) {
                return -1;
            }
        }
        return -1;
    }
}
