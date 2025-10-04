package net.polarbub.ftc.auto.vision;

import static net.polarbub.ftc.auto.vision.AlignSample.CAMERA_OFFSET;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import net.polarbub.ftc.MainBotHardware;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;
import net.polarbub.ftc.lib.FastIterativeOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Align Sample Test")
public class AlignSampleAutoTest extends LinearOpMode {
    private OpenCvCamera camera;
    private AlignSample pipeline;


    private MainBotHardware hardware;

    private boolean stopped;

    private static final double ALIGNMENT_THRESHOLD = 1.0; // Acceptable error in inches
    private static final double MAX_STRAFE_POWER = 0.28;    // Maximum motor power
    private static final double ALIGNMENT_KP = 0.02;      // Proportional gain constant

    @Override
    public void runOpMode() {
        hardware = new MainBotHardware(hardwareMap);
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "WC1"));
        pipeline = new AlignSample();

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        waitForStart();

        while (opModeIsActive()) {
            if(stopped) {
//                requestOpModeStop();
                stopMotors();
            }
            else if(!pipeline.isCentered()) {
                double xOffset = pipeline.getSmoothedXOffset() - CAMERA_OFFSET; // Get the raw xOffset from the pipeline
                double strafePower = ALIGNMENT_KP * xOffset;

                // Cap the motor power to avoid excessive speed
                strafePower = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, strafePower));

                telemetry.addData("Object Center X", pipeline.objectCenter != null ? pipeline.objectCenter.x : "null");
                telemetry.addData("Frame Width", pipeline.frameWidth);
                telemetry.addData("Raw X Offset", pipeline.getSmoothedXOffset());
                telemetry.addData("Camera Offset", CAMERA_OFFSET);
                telemetry.addData("Corrected Offset", pipeline.getSmoothedXOffset() - CAMERA_OFFSET);
                telemetry.addData("Is Center", pipeline.isCentered());



                if (xOffset > 0) {
                    hardware.leftFrontDrive.setPower(0.3);
                    hardware.rightFrontDrive.setPower(-0.3);
                    hardware.leftRearDrive.setPower(-0.3);
                    hardware.rightRearDrive.setPower(0.3);
                    telemetry.addData("Status", "Right");
                } else if (xOffset < 0) {
                    hardware.leftFrontDrive.setPower(-0.3);
                    hardware.rightFrontDrive.setPower(0.3);
                    hardware.leftRearDrive.setPower(0.3);
                    hardware.rightRearDrive.setPower(-0.3);
                    telemetry.addData("Status", "Left");
                } else {
                    telemetry.addData("Status", "Unable to Find");
                    stopMotors();
                    stopped = true;
                }
//                 Strafe based on the calculated power
            } else {
                stopMotors();
                stopped = true;
                telemetry.addData("Status", "Center");
            }

            telemetry.update();

//                telemetry.update();
        }





//    private void strafeRight(double power) {
//        hardware.leftFrontDrive.setPower(power);
//        hardware.leftRearDrive.setPower(-power);
//        hardware.rightFrontDrive.setPower(-power);
//        hardware.rightRearDrive.setPower(power);
//    }
//
//    private void strafeLeft(double power) {
//        hardware.leftFrontDrive.setPower(-power);
//        hardware.leftRearDrive.setPower(power);
//        hardware.rightFrontDrive.setPower(power);
//        hardware.rightRearDrive.setPower(-power);
//    }
//
//

    }
    private void stopMotors() {
        hardware.leftFrontDrive.setPower(0);
        hardware.leftRearDrive.setPower(0);
        hardware.rightFrontDrive.setPower(0);
        hardware.rightRearDrive.setPower(0);
    }
}