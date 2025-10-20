package net.polarbub.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import net.polarbub.ftc.MainBotConfig;

import static java.lang.Double.NaN;
import static net.polarbub.ftc.MainBotConfig.MoveConfig;

public class MainBotHardware {
    public final DcMotorExEx leftFrontDrive;
    public final DcMotorExEx rightFrontDrive;
    public final DcMotorExEx leftRearDrive;
    public final DcMotorExEx rightRearDrive;

    public final Encoder parallelEncoder0, parallelEncoder1, perpendicularEncoder;

    public final DcMotorExEx intakeDrive;
    public final DcMotorExEx launchDrive;
    public final DcMotorExEx launchDrive2;
    public final Servo launchServo;
    public final Servo launchServo2;

    public final RevColorSensorV3 colorSensor;

    public final Rev2mDistanceSensor leftDistance;
    public final Rev2mDistanceSensor rightFrontDistance;
    public final Rev2mDistanceSensor rightRearDistance;

    public final RevBlinkinLedDriver blinkin;

    public final HardwareMap hardwareMap;

    public final LynxModuleInitializer modules;

    public MainBotHardware(HardwareMap hardwareMap, boolean enableI2C) {
        this.hardwareMap = hardwareMap;
        modules = new LynxModuleInitializer(hardwareMap);
        LynxModuleInitializer.LynxModuleControllers parent = modules.parentModule;
        LynxModuleInitializer.LynxModuleControllers child = modules.childModule;

        leftFrontDrive = parent.createDcMotor(MainBotConfig.leftFrontDriveID);
        rightFrontDrive = parent.createDcMotor(MainBotConfig.rightFrontDriveID);
        leftRearDrive = parent.createDcMotor(MainBotConfig.leftRearDriveID);
        rightRearDrive = parent.createDcMotor(MainBotConfig.rightRearDriveID);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        intakeDrive = child.createDcMotor(MainBotConfig.intakeDriveID);
        launchDrive = child.createDcMotor(MainBotConfig.launchDriveID);
        launchDrive2 = child.createDcMotor(MainBotConfig.launchDrive2ID);

        intakeServo = createServo(MainBotConfig.intakeDriveID);
        intakeServo.setPosition(MainBotConfig.MoveConfig.restingServoPos);
        intakeServo2 = createServo(MainBotConfig.intakeDriveID);
        intakeServo2.setPosition(1 - MainBotConfig.MoveConfig.restingServoPos);

        blinkin = new RevBlinkinLedDriver(modules.parentModule.servoController, MainBotConfig.blinkinServoID);
        modules.parentModule.servoController.setServoPwmRange(MainBotConfig.blinkinServoID, new PwmControl.PwmRange(500, 2500));
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        parallelEncoder0 = parent.createEncoder(MainBotConfig.paraEncoder0ID);
        parallelEncoder1 = parent.createEncoder(MainBotConfig.paraEncoder1ID);
        perpendicularEncoder = parent.createEncoder(MainBotConfig.perpEncoderID);

        perpendicularEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        parallelEncoder0.setDirection(DcMotorSimple.Direction.REVERSE);
        parallelEncoder1.setDirection(DcMotorSimple.Direction.FORWARD);

        if(enableI2C) {
            colorSensor = new RevColorSensorV3(new LynxI2cDeviceSynchV2(hardwareMap.appContext, parent.module, MainBotConfig.colorSensorID),true);
//            leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.leftFrontDistID);
//            rightFrontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightFrontDistID);
//            rightRearDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightRearDistID);
            leftDistance = null;
            rightFrontDistance = null;
            rightRearDistance = null;
         } else {
            colorSensor = null;
            leftDistance = null;
            rightFrontDistance = null;
            rightRearDistance = null;
        }
    }

    public MainBotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, true);
    }
}
