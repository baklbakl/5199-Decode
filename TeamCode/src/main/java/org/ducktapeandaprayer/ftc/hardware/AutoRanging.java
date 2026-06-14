package org.ducktapeandaprayer.ftc.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.apache.commons.lang3.tuple.Pair;

import java.util.function.Function;

public class AutoRanging {
    public static double turretOffset = -52.0 /*mm*/ / 25.4;
    public static Vector2d goalTargetPos = new Vector2d(-66, 66);

    private final MainBotHardware.Container container;
    private final MainBotHardware hardware;
    private final Vector2d correctedGoalTargetPose;
    private final boolean enableDebugInfo;

    private boolean startOdoLauncher;

    private static double launchSpeedFromDist(double dist) {
        synchronized (MainBotHardware.launchSpeedMapping) {
            Pair<Double, Double> lowestPair = MainBotHardware.launchSpeedMapping.get(0);
            if(dist < lowestPair.getKey()) {
                //return Double.NaN;
                return lowestPair.getValue();
            }


            for(int i = 0; i < MainBotHardware.launchSpeedMapping.size(); i++) {
                Pair<Double, Double> p = MainBotHardware.launchSpeedMapping.get(i);
                if(dist == p.getKey()) {
                    return p.getValue();
                }

                if(dist < p.getKey()) {
                    //Previous one must have been greater, so we know we are between the two closest points,
                    // so we interpolate with a weighted average
                    Pair<Double, Double> last = MainBotHardware.launchSpeedMapping.get(i-1);
                    double deltaDist = p.getKey() - last.getKey();
                    double offset = dist - last.getKey();
                    double weightLast = (deltaDist - offset) / deltaDist;
                    double weightThis = offset / deltaDist;

                    return weightLast * last.getValue() + weightThis * p.getValue();
                }
            }
        }

        //return Double.NaN;
        return MainBotHardware.launchSpeedMapping.get(MainBotHardware.launchSpeedMapping.size() - 1).getValue();
    }


    public AutoRanging(MainBotHardware hardware, Function<Vector2d, Vector2d> invertIfBlue, boolean enableDebugInfo) {
        this.hardware = hardware;
        this.container = this.hardware.getContainer();
        this.correctedGoalTargetPose = invertIfBlue.apply(goalTargetPos);
        this.enableDebugInfo = enableDebugInfo;
    }

    public String tick(Pose2d robotPose) {
        StringBuilder telemetry = new StringBuilder();
        if(robotPose != null) {
            Pose2d turretPose = robotPose.plus(new Twist2d(new Vector2d(turretOffset, 0), 0));
            double distToGoal = correctedGoalTargetPose.minus(turretPose.position).norm();
            double autoSpeed = launchSpeedFromDist(distToGoal);

            if(enableDebugInfo) {
                telemetry.append("Dist to goal: ");
                telemetry.append(distToGoal);
                telemetry.append("\n");
                telemetry.append("Launcher Auto Speed Target: ");
                telemetry.append(autoSpeed);
                telemetry.append("\n");
            }

            if(container.launchDrive.getTargetVelocity() != 0 || startOdoLauncher) {
                startOdoLauncher = false;
                container.launchDrive.setVelocity(autoSpeed);
            }

            double fieldThetaToGoal = Math.atan2(correctedGoalTargetPose.y - turretPose.position.y, correctedGoalTargetPose.x - turretPose.position.x);
            double turretTheta = (fieldThetaToGoal - turretPose.heading.toDouble() + Math.PI * 2) % (Math.PI * 2);
            if(turretTheta > Math.PI) {
                turretTheta -= 2 * Math.PI;
            }

            if(enableDebugInfo) {
                telemetry.append("Turret tracking: ");
                telemetry.append(turretTracking);
                telemetry.append("\n");
                telemetry.append("Field angle to goal: ");
                telemetry.append(Math.toDegrees(fieldThetaToGoal));
                telemetry.append("\n");
                telemetry.append("Turret target theta: ");
                telemetry.append(Math.toDegrees(turretTheta));
                telemetry.append("\n");
            }

            if(turretTracking) {
                hardware.turretToHeading(turretTheta);
            }
        }

        return telemetry.toString();
    }

    public void setLauncherRunning(boolean state) {
        if(state) {
            startOdoLauncher = true;
        } else {
            container.launchDrive.setVelocity(0);
        }
    }

    public boolean turretTracking = false;

    public void setTurretTracking(boolean state) {
        turretTracking = state;
        if(!turretTracking) {
            hardware.turretToHeading(0);
        }
    }
}
