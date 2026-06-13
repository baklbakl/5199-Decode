package org.ducktapeandaprayer.ftc.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.auto.pidtopoint.Follower;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;

import static java.lang.Math.abs;

@TeleOp(name = "Test", group = MainBotConfig.opmodeGroupName)
public class Test extends FastIterativeOpMode {
    public static double distX = 0;
    public static double distY = 0;
    public static double distTheta = 0;
    private final static Pose2d startPoint = new Pose2d(0, 0, 0);

    private ElapsedTime runtime = new ElapsedTime();
    private ArrayList<VariableWatcher> variableWatchers = new ArrayList<>();
    private MainBotHardware hardware;
    private MainBotHardware.Container container;
    private Follower follower;

    public void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        container = hardware.getContainer();
        hardware.setOctoQuadPose(startPoint);
        follower = new Follower(hardware);
        follower.setPaused(true);

        try {
            variableWatchers.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), Gamepad.class.getField("circle"), Boolean::equals, (Boolean square, Boolean circle) -> {
                if(square == circle) {
                    return;
                }

                if(square) {
                    follower.setPaused(!follower.isPaused());
                } else if(circle && follower.isPaused()) {
                    hardware.setOctoQuadPose(startPoint);
                    follower.followTrajectory(List.of(
                            new Pose2d(distX, distY, Math.toRadians(distTheta))), true);
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        runtime.reset();
    }

    private void computeSetWheelSpeeds(double forward, double strafe, double turning) {

        double leftFrontPower  = forward + strafe + turning;
        double rightFrontPower = forward - strafe - turning;
        double leftBackPower   = forward - strafe + turning;
        double rightBackPower  = forward + strafe - turning;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max = Math.max(Math.max(abs(leftBackPower), Math.max(abs(leftFrontPower), abs(rightFrontPower))), abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        container.leftFrontDrive.setPower(leftFrontPower);
        container.leftRearDrive.setPower(leftBackPower);
        container.rightFrontDrive.setPower(rightFrontPower);
        container.rightRearDrive.setPower(rightBackPower);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop_internal() {
        container.modules.clearBulkCache();
        for(VariableWatcher watcher : variableWatchers) {
            watcher.tick();
        }

        telemetry.addLine(follower.tick(hardware.readOctoQuadLocalizerData()));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
    }
}
