package org.ducktapeandaprayer.ftc.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.ducktapeandaprayer.ftc.MainBotConfig;
import org.ducktapeandaprayer.ftc.hardware.LynxModuleInitializer;

@TeleOp(name="DemoBot (Small)", group = MainBotConfig.demoGroupName)
public class DemoBot extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LynxModuleInitializer hardware = LynxModuleInitializer.getOrCreate(hardwareMap, false);

        DcMotor leftDrive = hardware.parentModule.createDcMotor(0);
        DcMotor rightDrive = hardware.parentModule.createDcMotor(3);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y / 2;
            double turn = -gamepad1.right_stick_x / 2;
            if(turn != 0) {
                drive = 0;
            }
            double leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            double rightPower = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }
    }
}
