package frc8768.visionlib;

import java.util.List;

public interface Vision {
    /**
     * Get all targets.
     *
     * @return All targets in view.
     */
    List<?> getTargets();

    /**
     * Change the current pipeline.
     *
     * @param index Pipeline index, depends on your configuration.
     */
    void changePipeline(int index);

    /**
     * Get the Distance to target via trigonometry.
     *
     * @param mountAngle The amount of degrees back the limelight is.
     * @param mountHeight Height in inches from center of limelight lens to floor.
     * @param goalHeight Distance in inches from floor to center of target.
     * @param topY Get the top most location from the camera. See {@link #getMaxPointY() this method.}
     * @return Distance to target, returns -1 on fail.
     */
    double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY);

    /**
     * Sometimes, you may want to find the maximum Y for corners for the
     * intake to suck in the top of a cone, or cube. This function is helpful for that.
     *
     * @return The maximum Y point for all corners, or -1 if none is found
     */
    double getMaxPointY();
}
