package net.polarbub.ftc;

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

import static java.lang.Double.NaN;
import static net.polarbub.ftc.MainBotConfig.ArmConfig;

public class MainBotHardware {
//    public boolean enableI2C = false;

    public final DcMotorEx leftFrontDrive;
    public final DcMotorEx rightFrontDrive;
    public final DcMotorEx leftRearDrive;
    public final DcMotorEx rightRearDrive;

    public final Encoder parallelEncoder0, parallelEncoder1, perpendicularEncoder;

    public final DcMotorEx pivotDrive;
    public final DcMotorEx slideDrive;
    public final Servo intakeServo;
    public final Servo intakeServo2;

    public final RevColorSensorV3 colorSensor;

    public final Rev2mDistanceSensor leftDistance;
    public final Rev2mDistanceSensor rightFrontDistance;
    public final Rev2mDistanceSensor rightRearDistance;

    public final RevBlinkinLedDriver blinkin;

    public final HardwareMap hardwareMap;

    private LynxModule controlHub = null;
    private LynxModule expansionHub = null;

    private final LynxDcMotorController controlMotorController;
    private final LynxDcMotorController expansionMotorController;
    private final LynxServoController servoController;
    private final LynxAnalogInputController analogInputController;

    public MainBotHardware(HardwareMap hardwareMap, boolean initializeHardware, boolean enableI2C) {
        this.hardwareMap = hardwareMap;

        //FIX: Remove the hardware map and try to initialize all the hardware ourselves
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if(module.isParent()) {
                controlHub = module;
            } else {
                expansionHub = module;
            }
        }

        try {
            controlMotorController = new LynxDcMotorController(hardwareMap.appContext, controlHub);
            servoController = new LynxServoController(hardwareMap.appContext, controlHub);
            analogInputController = new LynxAnalogInputController(hardwareMap.appContext, controlHub);

            if(initializeHardware) {
                expansionMotorController = new LynxDcMotorController(hardwareMap.appContext, expansionHub);
                leftFrontDrive = createDcMotor(MainBotConfig.leftFrontDriveID);
                rightFrontDrive = createDcMotor(MainBotConfig.rightFrontDriveID);
                leftRearDrive = createDcMotor(MainBotConfig.leftRearDriveID);
                rightRearDrive = createDcMotor(MainBotConfig.rightRearDriveID);

                // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
                // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
                // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
                rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

                pivotDrive = createDcMotor(MainBotConfig.pivotDriveID);
                slideDrive = createDcMotor(MainBotConfig.slideDriveID);
//                pivotDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                slideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                runMotorToPos(pivotDrive, ArmConfig.pegPivotCounts, MainBotConfig.pivotPower);
//                runMotorToPos(slideDrive, 0, MainBotConfig.slidePower);

                intakeServo = createServo(MainBotConfig.intakeServoID);
                intakeServo.setPosition(ArmConfig.restingServoPos);
                intakeServo2 = createServo(MainBotConfig.intakeServo2ID);
                intakeServo2.setPosition(1 - ArmConfig.restingServoPos);

//                intakeServo.getController()

                blinkin = new RevBlinkinLedDriver(servoController, MainBotConfig.blinkinServoID);
                servoController.setServoPwmRange(MainBotConfig.blinkinServoID, new PwmControl.PwmRange(500, 2500));
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

                parallelEncoder0 = createEncoder(MainBotConfig.paraEncoder0ID);
                parallelEncoder1 = createEncoder(MainBotConfig.paraEncoder1ID);
                perpendicularEncoder = createEncoder(MainBotConfig.perpEncoderID);

                perpendicularEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
                parallelEncoder0.setDirection(DcMotorSimple.Direction.REVERSE);
                parallelEncoder1.setDirection(DcMotorSimple.Direction.FORWARD);

                if(enableI2C) {
                    colorSensor = new RevColorSensorV3(new LynxI2cDeviceSynchV2(hardwareMap.appContext, controlHub, MainBotConfig.colorSensorID),true);
                    leftDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.leftFrontDistID);
                    rightFrontDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightFrontDistID);
                    rightRearDistance = hardwareMap.get(Rev2mDistanceSensor.class, "D" + MainBotConfig.rightRearDistID);
                } else {
                    colorSensor = null;
                    leftDistance = null;
                    rightFrontDistance = null;
                    rightRearDistance = null;
                }
            } else {
                expansionMotorController = null;
                leftFrontDrive = null;
                rightFrontDrive = null;
                leftRearDrive = null;
                rightRearDrive = null;

                parallelEncoder0 = null;
                parallelEncoder1 = null;
                perpendicularEncoder = null;

                pivotDrive = null;
                slideDrive = null;

                blinkin = null;
                intakeServo = null;
                intakeServo2 = null;

                leftDistance = null;
                rightFrontDistance = null;
                rightRearDistance = null;

                colorSensor = null;
            }
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public MainBotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, true, true);
    }

    public LynxModule getControlHub() {
        return controlHub;
    }

    public LynxModule getExpansionHub() {
        return expansionHub;
    }

    public DcMotorEx createDcMotor(int id) {
        DcMotorEx motor;
        if(id > 3) {
            id -= 4;
            motor = new DcMotorImplEx(expansionMotorController, id, DcMotorSimple.Direction.FORWARD, MotorConfigurationType.getMotorType(GoBILDA5202Series.class));

        } else {
            motor = new DcMotorImplEx(controlMotorController, id, DcMotorSimple.Direction.FORWARD, MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
        }
        motor.setTargetPositionTolerance(MainBotConfig.motorPositionTolerance);
        return motor;
    }

    private ServoImplEx createServo(int id) {
        return new ServoImplEx(servoController, id, ServoConfigurationType.getStandardServoType());
    }

    public AnalogInput createAnalogInput(int id) {
        return new AnalogInput(analogInputController, id);
    }

    private Encoder createEncoder(int id) {
        return new OverflowEncoder(new RawEncoder(createDcMotor(id)));
    }


    public static void runMotorToPos(DcMotor motor, int targetPos, double power) {
        motor.setTargetPosition(targetPos);
        //This is cached for us, so it won't be updated if the power we are setting to is the same as the current one
        if(!Double.isNaN(power)) motor.setPower(power);
        //This is cached for us, so it won't be updated if we are already in run_to_position
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public static void runMotorToPos(DcMotor motor, int targetPos) {
        runMotorToPos(motor, targetPos, NaN);
    }
}