package org.ducktapeandaprayer.ftc.vision;

import org.openftc.apriltag.AprilTagPose;

public class AprilTagPoseEx extends AprilTagPose {
    public double groundDistance;
    public double pitch;

    public double updateLocalFields() {
        groundDistance = Math.cos(Math.atan(y / z)) * z;
        return groundDistance;
    }

    public AprilTagPoseEx() {
        updateLocalFields();
    }

    public AprilTagPoseEx(AprilTagPose other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        this.R = other.R;
        this.pitch = Math.atan2(-R.get(2, 0), Math.sqrt( Math.pow(R.get(2, 1), 2) +  Math.pow(R.get(2, 2), 2)));
        updateLocalFields();
    }
}
