package net.polarbub.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import net.polarbub.ftc.lib.OnCreateHandlers;
import net.polarbub.ftc.lib.SingletonHelper;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.jetbrains.annotations.NotNull;

public class MainBotHardware {
    public static final RevBlinkinLedDriver.BlinkinPattern wrongSpeedNoVision = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
    public static final RevBlinkinLedDriver.BlinkinPattern correctSpeedNoVision = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
    //
    // Dashboard options
    //
    public static PIDFCoefficients launcherCoefficients = new PIDFCoefficients(30, 0, 0, 14.4);
    public static int motorVelocityTolerance = 50;
    public static double restingServoPos = 0.5;
    public static double inServoPos = 0;
    public static double outServoPos = 1;
    public static double inIntakePower = 1;

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

    public static final int paraEncoder0ID = 3;
    public static final int paraEncoder1ID = 0;
    public static final int perpEncoderID = 1;

    public static final int moverServoID = 4;
    public static final int moverServo2ID = 5;
    public static final int blinkinServoID = 3;

    //The address of the expansion hub when attached via USB or RS485
    public static final int expansionHubAddress = 1;
    public static final int colorSensorI2CBus = 1;

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

    public final Servo moverServo;
    public final Servo moverServo2;

    public final RevColorSensorV3 launcherColorDistSensor;

    public final RevBlinkinLedDriver blinkin;
    public final LazyImu lazyImu;

    public final HardwareMap hardwareMap;
    public final WebcamName webcamName;
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
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        intakeDrive = child.createDcMotor(intakeDriveID);
        launchDrive = child.createDcMotor(launchDriveID);
        launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
        launchDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchDrive.setVelocity(0);
        launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moverServo = parent.createServo(moverServoID);
        moverServo.setPosition(restingServoPos);
        moverServo2 = parent.createServo(moverServo2ID);
        moverServo2.setPosition(restingServoPos);

        blinkin = new RevBlinkinLedDriver(modules.parentModule.servoController, blinkinServoID);
        modules.parentModule.servoController.setServoPwmRange(blinkinServoID, new PwmControl.PwmRange(500, 2500));
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

        launcherColorDistSensor = new RevColorSensorV3(LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, colorSensorI2CBus), true);
        webcamName = WebcamNameImpl.forSerialNumber(UsbScanner.getOrCreate(hardwareMap).webCamSerialNumber, () -> null, OnCreateHandlers.getEventLoopManger().get());
    }
}
