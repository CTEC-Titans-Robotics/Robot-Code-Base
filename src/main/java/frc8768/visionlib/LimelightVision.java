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
        return List.of(LimelightHelpers.getLatestResults(this.limelightName).targetingResults.targets_Fiducials);
    }

    @Override
    public void changePipeline(int index) {
        LimelightHelpers.setPipelineIndex(this.limelightName, index);
    }

    // TODO
    @Override
    public Object getBestTarget() {
        return LimelightHelpers.getLatestResults(this.limelightName).targetingResults;
    }

    @Override
    public double getDistanceToTarget(double mountAngle, double mountHeight, double goalHeight, boolean topY) {
        LimelightHelpers.LimelightTarget_Fiducial classifier = (LimelightHelpers.LimelightTarget_Fiducial) getBestTarget();
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
        LimelightHelpers.Results classifier = (LimelightHelpers.Results) getBestTarget();
        LimelightHelpers.LimelightTarget_Fiducial newClassifier = classifier.targets_Fiducials[0];
        if(newClassifier != null) {
            return newClassifier.ty + 0.25;
        }
        return -1;
    }

    @Override
    public int getTargetID() {
        LimelightHelpers.Results classifier = (LimelightHelpers.Results) getBestTarget();
        if(classifier == null) {
            return -1;
        }
        LimelightHelpers.LimelightTarget_Fiducial newClassifier = classifier.targets_Fiducials[0];
        if(newClassifier == null) {
            return -1;
        }
        return (int) newClassifier.fiducialID;
    }
}
