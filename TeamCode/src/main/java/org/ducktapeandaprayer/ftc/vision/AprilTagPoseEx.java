package org.ducktapeandaprayer.ftc.vision;

import org.openftc.apriltag.AprilTagPose;

public class AprilTagPoseEx extends AprilTagPose {
    public double groundDistance;

    public double updateGroundDistance() {
        groundDistance = Math.cos(Math.atan(y / z)) * z;
        return groundDistance;
    }

    public AprilTagPoseEx() {
        updateGroundDistance();
    }

    public AprilTagPoseEx(AprilTagPose other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        this.R = other.R;
        updateGroundDistance();
    }
}
