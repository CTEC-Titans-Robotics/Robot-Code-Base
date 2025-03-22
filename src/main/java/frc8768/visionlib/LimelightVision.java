package frc8768.visionlib;

import edu.wpi.first.math.geometry.Transform3d;
import frc8768.visionlib.helpers.LimelightHelpers;

import java.util.List;

public class LimelightVision implements Vision {
    private final String limelightName;
    private Transform3d cameraTransform;

//    public LimelightVision(String name, Transform3d transform) {
    public LimelightVision(String name) {

        this.limelightName = name;
        //this.cameraTransform = transform;
    }

    @Override
    public List<LimelightHelpers.LimelightTarget_Fiducial> getTargets() {
        return List.of(LimelightHelpers.getLatestResults(this.limelightName).targets_Fiducials);
    }

    @Override
    public void changePipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.limelightName, index);
    }

    @Override
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        List<LimelightHelpers.LimelightTarget_Fiducial> targets = getTargets();
        if(targets.isEmpty()) {
            return -1;
        }

        LimelightHelpers.LimelightTarget_Fiducial classifier = targets.get(0);
        if(classifier != null) {
            double angleToGoalDegrees = mountAngle + (topY ? getMaxPointY() : classifier.ty);
            double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

            double distance = (goalHeight - mountHeight) / Math.tan(angleToGoalRadians);
            if(distance <= 0) {
                return -1;
            }
            return distance;
        }
        return -1;
    }

    @Override
    public double getMaxPointY() {
        LimelightHelpers.LimelightTarget_Fiducial newClassifier = getTargets().get(0);
        if(newClassifier != null) {
            return newClassifier.ty + 0.25;
        }
        return -1;
    }
}
