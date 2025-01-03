package frc8768.visionlib;

import frc8768.visionlib.helpers.LimelightHelpers;

import java.util.List;

public class LimelightVision extends Vision {
    private final String limelightName;

    public LimelightVision(String name) {
        this.limelightName = name;
    }

    @Override
    public List<LimelightHelpers.LimelightTarget_Fiducial> getTargets() {
        return List.of(LimelightHelpers.getLatestResults(this.limelightName).targetingResults.targets_Fiducials);
    }

    @Override
    public void changePipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.limelightName, index);
    }

    @Override
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        LimelightHelpers.LimelightTarget_Fiducial classifier = getTargets().get(0);
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
        LimelightHelpers.LimelightTarget_Fiducial newClassifier = getTargets().get(0);
        if(newClassifier != null) {
            return newClassifier.ty + 0.25;
        }
        return -1;
    }

    @Override
    public int getTargetID() {
        LimelightHelpers.LimelightTarget_Fiducial newClassifier = getTargets().get(0);
        if(newClassifier == null) {
            return -1;
        }
        return (int) newClassifier.fiducialID;
    }
}
