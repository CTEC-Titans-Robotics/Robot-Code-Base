package frc8768.visionlib.multicam;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc8768.visionlib.Vision;

import java.util.List;

/**
 * For Robots that want a joined vision system, say 4 cameras on 4 corners
 *
 * TODO: Add methods for really fancy math and functions, such as multi-camera targeting/field recognition
 */
public interface MultiCamVision {
    /**
     * Add a camera to the vision system
     *
     * @param camName Name of the camera in associated system
     * @param pos Transform relative to Robot center
     */
    void addCamera(String camName, Transform3d pos);

    /**
     * Get targets from camera
     *
     * @return targets if any, null for none.
     */
    List<?> getTargetsForCam(String camName);

    /**
     * Get targets for all cameras
     *
     * @return targets if any, null for none.
     */
    List<?> getTargets();

    /**
     * Change the current pipeline for camera
     *
     * @param index Pipeline index, depends on your configuration.
     */
    void changePipeline(String camName, int index);

    /**
     * Get pose of target relative to Robot
     */
    Transform3d getTargetPose(String camName);
}
