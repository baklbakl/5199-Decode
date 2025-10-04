package net.polarbub.ftc.tele;

import android.graphics.Color;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV2;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Sensor Reader", group = MainBotConfig.opmodeGroupName)
public class SensorReader extends FastIterativeOpMode {
    private DcMotorEx drive0 = null;
    private DcMotorEx drive1 = null;
    private DcMotorEx drive2 = null;
    private DcMotorEx drive3 = null;
    private DcMotorEx drive4 = null;
    private DcMotorEx drive5 = null;
    private DcMotorEx drive6 = null;
    private DcMotorEx drive7 = null;
    private AnalogInput analog0 = null;
    private AnalogInput analog1 = null;
    private AnalogInput analog2 = null;
    private AnalogInput analog3 = null;
    private RevColorSensorV3 color0 = null;
    private Rev2mDistanceSensor dist0 = null;
    private Rev2mDistanceSensor dist1 = null;
    private Rev2mDistanceSensor dist2 = null;

    private ElapsedTime runtime = new ElapsedTime();
    List<LynxModule> allHubs;

    private static I2cDeviceSynch getI2cDeviceSynchFromSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
        if (simple instanceof I2cDeviceSynch) { return (I2cDeviceSynch) simple; }
        else { return new I2cDeviceSynchImplOnSimple(simple, isSimpleOwned); }
    }

    protected void init_internal() {
        //telemetry.addData("Status", "Initialized");
        MainBotHardware hardware = new MainBotHardware(hardwareMap, false, false);

        try {
            new ServoImplEx(new LynxServoController(hardwareMap.appContext, hardware.getControlHub()), 0, ServoConfigurationType.getStandardServoType()).setPosition(0.5);
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }

        drive0 = hardware.createDcMotor(0);
        drive1 = hardware.createDcMotor(1);
        drive2 = hardware.createDcMotor(2);
        drive3 = hardware.createDcMotor(3);
        drive4 = hardware.createDcMotor(4);
        drive5 = hardware.createDcMotor(5);
        drive6 = hardware.createDcMotor(6);
        drive7 = hardware.createDcMotor(7);

        drive0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        analog0 = hardware.createAnalogInput(0);
        analog1 = hardware.createAnalogInput(1);
        analog2 = hardware.createAnalogInput(2);
        analog3 = hardware.createAnalogInput(3);

        color0 = new RevColorSensorV3(new LynxI2cDeviceSynchV2(hardwareMap.appContext, hardware.getControlHub(), MainBotConfig.colorSensorID), true);

        dist0 = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.leftFrontDistID);
        dist1 = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightFrontDistID);
        dist2 = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightRearDistID);

//        dist1 = new Rev2mDistanceSensor(getI2cDeviceSynchFromSimple(new LynxI2cDeviceSynchV2(hardwareMap.appContext, hardware.getControlHub(), MainBotConfig.rightFrontDistID), simpleOwned), deviceOwned);
//        dist2 = new Rev2mDistanceSensor(getI2cDeviceSynchFromSimple(new LynxI2cDeviceSynchV2(hardwareMap.appContext, hardware.getControlHub(), MainBotConfig.rightRearDistID), simpleOwned), deviceOwned);


        //SPEEEEEEEED Mode. We put it in manual mode. https://github.com/first-tech-challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptMotorBulkRead.java
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    protected void init_loop_internal() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    protected void start_internal() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    protected void loop_internal() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        NormalizedRGBA color = color0.getNormalizedColors();

        telemetry.addData(
                "Status", "Run Time: " + runtime.toString() +
                "\nEncoder 0: " + drive0.getCurrentPosition() +
                "\nEncoder 1: " + drive1.getCurrentPosition() +
                "\nEncoder 2: " + drive2.getCurrentPosition() +
                "\nEncoder 3: " + drive3.getCurrentPosition() +
                "\nEncoder 4: " + drive4.getCurrentPosition() +
                "\nEncoder 5: " + drive5.getCurrentPosition() +
                "\nEncoder 6: " + drive6.getCurrentPosition() +
                "\nEncoder 7: " + drive7.getCurrentPosition() +
                "\nAnalog 0: " + analog0.getVoltage() +
                "\nAnalog 1: " + analog1.getVoltage() +
                "\nAnalog 2: " + analog2.getVoltage() +
                "\nAnalog 3: " + analog3.getVoltage() +
                "\nColor 0: {" + color.red + ", " + color.green + ", " + color.blue + ", " + color.alpha + '}' +
                "\nColor dist 0: " + color0.getDistance(DistanceUnit.INCH) +
                "\nDist 0: " + dist0.getDistance(DistanceUnit.INCH) +
                "\nDist 1: " + dist1.getDistance(DistanceUnit.INCH) +
                "\nDist 2: " + dist2.getDistance(DistanceUnit.INCH)
        );
        telemetry.update();
        try {
            Thread.sleep(1);
        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    protected void stop_internal() {
    }
}
