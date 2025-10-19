package net.polarbub.ftc.tele;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.LynxModuleInitializer;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test", group = MainBotConfig.opmodeGroupName)
public class Test extends FastIterativeOpMode {
    public static double thresh = 0.8;

    private ElapsedTime runtime = new ElapsedTime();
    private MainBotHardware hardware;

    public void init_internal() {
        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
//        allHubs = hardwareMap.getAll(LynxModule.class);
        hardware = new MainBotHardware(hardwareMap, false);
        hardware.modules.parentModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        hardware.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    public void init_loop_internal() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    public void start_internal() {
        runtime.reset();
        hardware.intakeServo.setPosition(MainBotConfig.ArmConfig.grabServoPos);
        hardware.intakeServo2.setPosition(1-MainBotConfig.ArmConfig.grabServoPos);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    public void loop_internal() {
        hardware.modules.parentModule.module.clearBulkCache();
        hardware.modules.childModule.module.clearBulkCache();

        if(hardware.colorSensor.getDistance(DistanceUnit.INCH) < thresh) {
            hardware.intakeServo.setPosition(MainBotConfig.ArmConfig.restingServoPos);
            hardware.intakeServo2.setPosition(MainBotConfig.ArmConfig.restingServoPos);
        };
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public void stop_internal() {
    }
}
