package frc8768.visionlib.multicam;

import edu.wpi.first.math.geometry.Transform3d;
import frc8768.robot.util.LogUtil;
import frc8768.visionlib.PhotonVision;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Currently only supports 1 type of processor per joined system
 */
public class PhotonMultiCam implements MultiCamVision {
    final Map<String, PhotonCameraStorage> cameras = new HashMap<>();

    @Override
    public void addCamera(String camName, Transform3d pos) {
        PhotonCameraStorage storage = cameras.put(camName,
                new PhotonCameraStorage(new PhotonVision(camName), pos));

        if(storage != null)
            LogUtil.LOGGER.warning("Camera " + camName + " already existed.");
    }

    @Override
    public List<PhotonTrackedTarget> getTargetsForCam(String camName) {
        PhotonCameraStorage storage = cameras.get(camName);
        return storage.camera.getTargets();
    }

    @Override
    public void changePipeline(String camName, int index) {
        PhotonCameraStorage storage = cameras.get(camName);
        if(storage != null)
            storage.camera.changePipeline(index);
    }

    @Override
    public Transform3d getTargetPose(String camName) {
        PhotonCameraStorage storage = cameras.get(camName);

        PhotonTrackedTarget target = storage.camera.getTargets().get(0);
        if(target == null) {
            return null;
        }
        return target.getBestCameraToTarget().plus(storage.mountPos.inverse()); // TODO test this
    }

    @Override
    public List<PhotonTrackedTarget> getTargets() {
        ArrayList<PhotonTrackedTarget> targets = new ArrayList<>();
        for(String name : cameras.keySet())
            targets.addAll(getTargetsForCam(name));

        return targets;
    }

    record PhotonCameraStorage(PhotonVision camera, Transform3d mountPos) {}
}
