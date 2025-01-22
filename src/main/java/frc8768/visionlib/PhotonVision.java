package frc8768.visionlib;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import java.util.Collections;
import java.util.List;

/**
 * Impl for photonvision for Vision-related things.
 */
public class PhotonVision extends Vision {
    private final PhotonCamera camera;

    /**
     * @param type One of {@link Type}
     *             NOTE: Limelight 3 is pi for now.
     */
    public PhotonVision(Type type) {
        camera = new PhotonCamera(type.name);
    }

    /**
     * Get all targets.
     *
     * @return All targets in view.
     */
    public List<PhotonPipelineResult> getTargets() {
        if(!camera.getAllUnreadResults().isEmpty()) {
            return camera.getAllUnreadResults();
        }
        return List.of();
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
     * Get the Distance to target via trigonometry.
     *
     * @param mountAngle The amount of degrees back the limelight is.
     * @param mountHeight Height in inches from center of limelight lens to floor.
     * @param goalHeight Distance in inches from floor to center of target.
     * @param topY Get the top most location from the camera. See {@link #getMaxPointY() this method.}
     * @return Distance to target, returns -1 on fail.
     */
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        PhotonTrackedTarget target;

        if(getTargets().isEmpty()) {
            return -1;
        }
        target = getTargets().get(0).getBestTarget();

        if(target != null) {
            double angleToGoalDegrees = mountAngle + (topY ? getMaxPointY() : target.getBestCameraToTarget().getY());
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

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
     * @return The maximum Y point for all corners, or -1 if none is found
     */
    public double getMaxPointY() {
        PhotonTrackedTarget target;

        if(getTargets().isEmpty()) {
            return -1;
        }
        target = getTargets().get(0).getBestTarget();

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
        PhotonTrackedTarget target;

        if(getTargets().isEmpty()) {
            return -1;
        }
        target = getTargets().get(0).getBestTarget();

        if(target != null) {
            return target.getFiducialId();
        }
        return -1;
    }

    /**
     * Type of {@link PhotonCamera}
     */
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
