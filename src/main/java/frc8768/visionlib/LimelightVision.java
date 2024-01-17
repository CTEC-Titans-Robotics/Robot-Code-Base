package frc8768.visionlib;

import frc8768.visionlib.helpers.LimelightHelpers;

import java.util.List;

public class LimelightVision extends Vision {
    private final String limelightName;

    public LimelightVision(String name) {
        this.limelightName = name;
    }

    @Override
    public List<Object> getTargets() {
        return List.of(LimelightHelpers.getLatestResults(this.limelightName).targetingResults.targets_Classifier);
    }

    @Override
    public void changePipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.limelightName, index);
    }

    // TODO
    @Override
    public Object getBestTarget() {
        LimelightHelpers.LimelightTarget_Classifier[] classifier = LimelightHelpers.getLatestResults(this.limelightName).targetingResults.targets_Classifier;
        if(classifier.length > 0) {
            return classifier[0];
        }

        return null;
    }

    @Override
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        LimelightHelpers.LimelightTarget_Classifier classifier = (LimelightHelpers.LimelightTarget_Classifier) getBestTarget();
        if(classifier != null) {
            double angleToGoalDegrees = mountAngle + (topY ? getMaxPointY() : classifier.ty);
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

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
        LimelightHelpers.LimelightTarget_Classifier classifier = (LimelightHelpers.LimelightTarget_Classifier) getBestTarget();
        if(classifier != null) {
            return classifier.ty + 0.08255;
        }
        return -1;
    }
}
