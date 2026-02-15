package org.ducktapeandaprayer.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import org.ducktapeandaprayer.ftc.lib.OnCreateHandlers;
import org.ducktapeandaprayer.ftc.lib.SingletonHelper;
import org.apache.commons.lang3.tuple.MutablePair;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MainBotHardware {
    //
    // Dashboard options
    //
    public static PIDFCoefficients launcherCoefficients = new PIDFCoefficients(60, 0, 0, 14.5);
    public static int motorVelocityTolerance = 30;
    public static double moverDrivePower = 1;
    public static double moverDriveBallInPower = 0.7; //Was 0.85
    public static double moverDriveAutoPower = 0.85; //Was 0.85
    public static double kickerServoUpPos = 0.5;
    public static double kickerServoDownPos = 0.32;
    public static double inIntakePower = 1;
    public static long teleOpLauncherNoVisionVelocity = 1600;
    public static int launchRequiredSpeedSamples = 10;
    public static double parentResistorMultiplier = 3.19;

    //MUST BE IN ORDER OF DISTANCE!!!!!
    //MAKE SURE TO SYNCHRONIZE WHEN USING!
    public static final List<MutablePair<Double, Double>> launchSpeedMapping = new ArrayList<>(Arrays.asList(
            new MutablePair<>(25.0, -1310.0),
            new MutablePair<>(33.0, -1340.0),
            new MutablePair<>(40.0, -1370.0),
            new MutablePair<>(47.0, -1420.0),
            new MutablePair<>(55.0, -1460.0),
            new MutablePair<>(60.0, -1500.0),
            new MutablePair<>(70.0, -1550.0),
            new MutablePair<>(78.0, -1630.0)
    ));

    //
    // Options
    //
    public static class Patterns {
        //Offcolor lets us know that the blinkin is working when we press init
        public static final RevBlinkinLedDriver.BlinkinPattern off = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE;
        public static final RevBlinkinLedDriver.BlinkinPattern manualLaunchControl = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        public static final RevBlinkinLedDriver.BlinkinPattern correctSpeed = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        public static final RevBlinkinLedDriver.BlinkinPattern wrongSpeedVisionLocked = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        public static final RevBlinkinLedDriver.BlinkinPattern outOfLaunchRange = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        public static final RevBlinkinLedDriver.BlinkinPattern tagNotFound = RevBlinkinLedDriver.BlinkinPattern.RED;
        public static final RevBlinkinLedDriver.BlinkinPattern wrongSpeedNoVisionPattern = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
        public static final RevBlinkinLedDriver.BlinkinPattern correctSpeedNoVision = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
    }

    //
    // Hardware ports
    //
    // IMU orientation
    public static final RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
    public static final RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static final int leftFrontDriveID = 0;
    public static final int rightFrontDriveID = 1;
    public static final int leftRearDriveID = 2;
    public static final int rightRearDriveID = 3;

    public static final int intakeDriveID = 0;
    public static final int launchDriveID = 3;
    public static final int moverDriveID = 1;

    public static final int paraEncoder0ID = 0;
    public static final int paraEncoder1ID = 3;
    public static final int perpEncoderID = 1;

    public static final int kickerServoID = 1;
    public static final int blinkinServoID = 0;

    //The address of the expansion hub when attached via USB or RS485
    public static final int expansionHubAddress = 1;
//    public static final int colorSensorI2CBus = 1;
//    public static final int colorSensor2I2CBus = 2;

    //
    // Actual hardware
    //

    public final DcMotorExEx leftFrontDrive;
    public final DcMotorExEx rightFrontDrive;
    public final DcMotorExEx leftRearDrive;
    public final DcMotorExEx rightRearDrive;

    public final Encoder parallelEncoder0, parallelEncoder1, perpendicularEncoder;

    public final DcMotorExEx intakeDrive;
    public final DcMotorExEx launchDrive;
    public final DcMotorExEx moverDrive;

    public final Servo kickerServo;

    public final VoltageSensor parentVoltageSensor;

    //True is not pressed, false is
    public final DigitalChannel launcherSwitch;

    public final RevBlinkinLedDriver blinkin;
    public final LazyImu lazyImu;

    public final HardwareMap hardwareMap;
    public final WebcamName webcamName;
    public final LynxModuleInitializer modules;


    public static MainBotHardware getOrCreate(HardwareMap hardwareMap) {
        MainBotHardware ts = SingletonHelper.getOrCreate(MainBotHardware.class, () -> new MainBotHardware(hardwareMap));
        if(ts.modules != LynxModuleInitializer.getOrCreate(hardwareMap)) {
//            ts = new MainBotHardware(hardwareMap);
            SingletonHelper.clear(MainBotHardware.class);
        }
        return SingletonHelper.getOrCreate(MainBotHardware.class, () -> new MainBotHardware(hardwareMap));
    }

    public static void clearInstance() {
        SingletonHelper.clear(MainBotHardware.class);
    }

    private MainBotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        RobotLog.ii("MainBotHardware", "Initializing mainbothardware");

        modules = LynxModuleInitializer.getOrCreate(hardwareMap);
        if(modules.childModule == null) {
            throw new RuntimeException("No child module found. A child module is required for MainBotHardware");
        }

        LynxModuleInitializer.LynxModuleControllers parent = modules.parentModule;
        LynxModuleInitializer.LynxModuleControllers child = modules.childModule;

        leftFrontDrive = parent.createDcMotor(leftFrontDriveID);
        rightFrontDrive = parent.createDcMotor(rightFrontDriveID);
        leftRearDrive = parent.createDcMotor(leftRearDriveID);
        rightRearDrive = parent.createDcMotor(rightRearDriveID);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeDrive = child.createDcMotor(intakeDriveID);
        moverDrive = child.createDcMotor(moverDriveID);

        launchDrive = child.createDcMotor(launchDriveID);
        launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        launchDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchDrive.setVelocity(0);
        launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        kickerServo = child.createServo(kickerServoID);
        kickerServo.setPosition(MainBotHardware.kickerServoDownPos);

        blinkin = new RevBlinkinLedDriver(modules.childModule.servoController, blinkinServoID);
        modules.childModule.servoController.setServoPwmRange(blinkinServoID, new PwmControl.PwmRange(500, 2500));
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        parallelEncoder0 = parent.createEncoder(paraEncoder0ID);
        parallelEncoder1 = parent.createEncoder(paraEncoder1ID);
        perpendicularEncoder = parent.createEncoder(perpEncoderID);

        perpendicularEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        parallelEncoder0.setDirection(DcMotorSimple.Direction.FORWARD);
        parallelEncoder1.setDirection(DcMotorSimple.Direction.REVERSE);

        LynxModuleImuType imuType = parent.module.getImuType();

        if(imuType == LynxModuleImuType.BNO055) {
            //This is kind of jank, but it works.
            lazyImu = new LazyImu() {
                private IMU imu = null;

                @NotNull
                @Override
                public IMU get() {
                    if(imu == null) {
                        imu = new LynxEmbeddedBNO055IMUNew(LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, 0), true);
                        if(imu.initialize(new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection)))) {
                            imu.resetYaw();
                        } else {
                            throw new RuntimeException("Failed to init IMU");
                        }
                    }

                    return imu;
                }
            };
        } else {
            throw new RuntimeException("Acceptable type of IMU in parent module: \"" + imuType + "\". Only \"" + LynxModuleImuType.BNO055 + "\" is acceptable");
        }

//        launcherColorDistSensor = new RevColorSensorV3(LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, colorSensorI2CBus), true);
//        launcherColorDistSensor2 = new RevColorSensorV3(LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, colorSensor2I2CBus), true);

        SerialNumber webCamSerial = UsbScanner.getOrCreate(hardwareMap).webCamSerialNumber;

        if(webCamSerial == null) {
            webcamName = null;
//            throw new RuntimeException("No webcam found and webcam is required for MainBotHardware");
        } else {
            webcamName = WebcamNameImpl.forSerialNumber(webCamSerial, () -> null, OnCreateHandlers.getEventLoopManger().get());
        }

        launcherSwitch = child.createDigitalChannel(0);
        parentVoltageSensor = new ProxyVoltageSensor(parent.createAnalogInput(0), parentResistorMultiplier);
        hardwareMap.put("ProxyVoltageSensor", parentVoltageSensor);
        hardwareMap.voltageSensor.put("ProxyVoltageSensor", parentVoltageSensor);
    }
}
