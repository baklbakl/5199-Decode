package net.polarbub.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.*;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.lib.RequireSingleInstance;
import net.polarbub.ftc.lib.SingletonHelper;
import org.jetbrains.annotations.NotNull;

import java.lang.ref.WeakReference;

public class MainBotHardware {
    public final DcMotorExEx leftFrontDrive;
    public final DcMotorExEx rightFrontDrive;
    public final DcMotorExEx leftRearDrive;
    public final DcMotorExEx rightRearDrive;

    public final Encoder parallelEncoder0, parallelEncoder1, perpendicularEncoder;

    public final DcMotorExEx intakeDrive;
    public final DcMotorExEx launchDrive;

    public final Servo moverServo;
    public final Servo moverServo2;

    //ADD: Color sensor support

    public final RevBlinkinLedDriver blinkin;
    public final LazyImu lazyImu;

    public final HardwareMap hardwareMap;
    //ADD: Webcam support
    public final LynxModuleInitializer modules;

    public static MainBotHardware getOrCreate(HardwareMap hardwareMap) {
        return SingletonHelper.getOrCreate(MainBotHardware.class, () -> new MainBotHardware(hardwareMap));
    }

    private MainBotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        modules = LynxModuleInitializer.getOrCreate(hardwareMap);
        if(modules.childModule == null) {
            throw new RuntimeException("No child module found. A child module is required for MainBotHardware");
        }

        LynxModuleInitializer.LynxModuleControllers parent = modules.parentModule;
        LynxModuleInitializer.LynxModuleControllers child = modules.childModule;

        LynxModuleImuType imuType = parent.module.getImuType();

        if(imuType == LynxModuleImuType.BNO055) {
            lazyImu = () -> {
                I2cDeviceSynchSimple result = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, 0);
                return new LynxEmbeddedBNO055IMUNew(result, true);
            };
        } else {
            throw new RuntimeException("Acceptable type of IMU in parent module: \"" + imuType + "\". Only \"" + LynxModuleImuType.BNO055 + "\" is acceptable");
        }

        leftFrontDrive = parent.createDcMotor(MainBotConfig.leftFrontDriveID);
        rightFrontDrive = parent.createDcMotor(MainBotConfig.rightFrontDriveID);
        leftRearDrive = parent.createDcMotor(MainBotConfig.leftRearDriveID);
        rightRearDrive = parent.createDcMotor(MainBotConfig.rightRearDriveID);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        intakeDrive = child.createDcMotor(MainBotConfig.intakeDriveID);
        launchDrive = child.createDcMotor(MainBotConfig.launchDriveID);
        launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moverServo = parent.createServo(MainBotConfig.moverServoID);
        moverServo.setPosition(MainBotConfig.MoveConfig.restingServoPos);
        moverServo2 = parent.createServo(MainBotConfig.moverServo2ID);
        moverServo2.setPosition(MainBotConfig.MoveConfig.restingServoPos);

        blinkin = new RevBlinkinLedDriver(modules.parentModule.servoController, MainBotConfig.blinkinServoID);
        modules.parentModule.servoController.setServoPwmRange(MainBotConfig.blinkinServoID, new PwmControl.PwmRange(500, 2500));
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        parallelEncoder0 = parent.createEncoder(MainBotConfig.paraEncoder0ID);
        parallelEncoder1 = parent.createEncoder(MainBotConfig.paraEncoder1ID);
        perpendicularEncoder = parent.createEncoder(MainBotConfig.perpEncoderID);

        perpendicularEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        parallelEncoder0.setDirection(DcMotorSimple.Direction.FORWARD);
        parallelEncoder1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
