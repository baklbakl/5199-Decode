package org.ducktapeandaprayer.ftc.tele;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.FastIterativeOpMode;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.DoubleVariableWatcher;
import org.ducktapeandaprayer.ftc.lib.variablewatcher.VariableWatcher;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test", group = MainBotConfig.opmodeGroupName)
public class Test extends FastIterativeOpMode {
    public static double kP = 0.3;
    public static double kS = 1;
    public static double doneThresh = 1;
    public static double kDfake = -0.02;
    public static double kDfakePPower = 1.5;

    private final static List<Pose2d> waypoints = List.of(
            new Pose2d(25, 0, 0),
            new Pose2d(25, 25, 0),
            new Pose2d(0, 0, 0)
    );

    private final static Pose2d startPoint = new Pose2d(0, 0, 0);
    private final static Pose2d targetPose = new Pose2d(25, 0, 0);

    private ElapsedTime runtime = new ElapsedTime();
    private ArrayList<VariableWatcher> variableWatchers = new ArrayList<>();
    private MainBotHardware hardware;
    private MainBotHardware.Container container;
    private boolean run = false;

    public void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        container = hardware.getContainer();
        hardware.setOctoQuadPose(startPoint);
        container.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        container.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            variableWatchers.add(new DoubleVariableWatcher<>(gamepad1, Gamepad.class.getField("square"), Gamepad.class.getField("circle"), Boolean::equals, (Boolean square, Boolean circle) -> {
                if(square == circle) {
                    return;
                }

                if(square) {
                    run = !run;
                    if(!run) {
                        container.leftFrontDrive.setPower(0);
                        container.rightFrontDrive.setPower(0);
                        container.leftRearDrive.setPower(0);
                        container.rightRearDrive.setPower(0);
                    }
                } else if(circle && !run) {
                    hardware.setOctoQuadPose(startPoint);
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

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop_internal() {
        container.modules.clearBulkCache();
        for(VariableWatcher watcher : variableWatchers) {
            watcher.tick();
        }

        OctoQuad.LocalizerDataBlock dataBlock = hardware.readOctoQuadLocalizerData();
        Pose2d robotPose = MainBotHardware.makeOctoQuadPose(dataBlock);
        Pose2d poseError = targetPose.minusExp(robotPose);
        double posError = poseError.position.x;
        //Target vel is zero
        double velError = dataBlock.velX_mmS / 25.4 - 0;
        double power = posError * kP + Math.signum(posError) * kS + kDfake * velError / Math.pow(Math.abs(posError), kDfakePPower);
        power /= container.voltageSensor.getVoltage();

        if(Math.abs(posError) < doneThresh) {
            power = 0;
        }

        if(run) {
            container.leftFrontDrive.setPower(power);
            container.rightFrontDrive.setPower(power);
            container.leftRearDrive.setPower(power);
            container.rightRearDrive.setPower(power);
        }

        telemetry.addData("run", run);
        telemetry.addData("Pose error", poseError);
        telemetry.addData("pos error", posError);
        telemetry.addData("vel error", velError);
        telemetry.addData("power", power);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
    }
}
