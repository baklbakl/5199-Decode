package org.ducktapeandaprayer.ftc.hardware;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuadImpl;
import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.hardware.lynx.commands.core.LynxFirmwareVersionManager;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.lib.SingletonHelper;
import org.apache.commons.lang3.tuple.MutablePair;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class MainBotHardware {
    //
    // Dashboard options
    //
    public static PIDFCoefficients launcherCoefficients = new PIDFCoefficients(60, 0, 0, 11);
    public static int motorVelocityTolerance = 30;
    public static double transferInPower = 1;
    public static double rampShootPos = 0.47;
    public static double rampLastBallPos = 0.40;
    public static double rampDownPos = 0.545;
    public static double intakeInPower = 1;
    public static double launcherPresetVelocity = 2120;
    public static double resistorMultiplier = 3.38;
    public static double turretMaxHeading = 2.3;
    public static double turretCenter = 0.495;
    public static double shootTransferOnDelay = 0.3;
    public static double singleShootHighRampDelay = 0.4;
    //This number comes from the pinpoint driver
    public static float encoderTicksPerMM = 13.26291192f;
    public static double axonRange = 320;


    //MUST BE IN ORDER OF DISTANCE!!!!!
    //MAKE SURE TO SYNCHRONIZE WHEN USING!
    public static final List<MutablePair<Double, Double>> launchSpeedMapping = new ArrayList<>(Arrays.asList(
            new MutablePair<>(47.0, 2100.0),
            new MutablePair<>(54.0, 2100.0),
            new MutablePair<>(64.0, 2150.0),
            new MutablePair<>(73.0, 2230.0),
            //new MutablePair<>(80.0, 2340.0),
            new MutablePair<>(80.0, 2260.0),
            new MutablePair<>(90.0, 2340.0),
            new MutablePair<>(100.0, 2520.0),
            new MutablePair<>(110.0, 2660.0),
            new MutablePair<>(120.0, 2780.0)
            ));

    //
    // Options
    //

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

    public static final int intakeDriveID = 1;
    public static final int launchDriveID = 0;
    public static final int moverDriveID = 2;

    public static final int xEncoderIndex = 0;
    public static final int yEncoderIndex = 1;

    public static final int turretServoID = 0;
    public static final int kickerServoID = 1;

    public static final int intakeBBID = 2;
    public static final int transferMiddleBBID = 6;
    public static final int transferEndBBID = 4;

    //The address of the expansion hub when attached via USB or RS485
    public static final int expansionHubAddress = 1;

    //
    // Actual hardware
    //
    private Container container = null;
    public Container getContainer() {
        if(container == null) {
            container = new Container(hardwareMap, modules, octoQuad);
        }
        return container;
    }

    public class Container {
        public final DcMotorExEx leftFrontDrive;
        public final DcMotorExEx rightFrontDrive;
        public final DcMotorExEx leftRearDrive;
        public final DcMotorExEx rightRearDrive;

        public final DcMotorExEx intakeDrive;
        public final DcMotorExEx launchDrive;
        public final DcMotorExEx transferDrive;

        public final Servo rampServo;
        public final Servo turretServo;

        public final ProxyVoltageSensor voltageSensor;

        public final LazyImu lazyImu;

        public final BeamBreak intakeBB;
        public final BeamBreak transferMiddleBB;
        //Under shooter
        public final BeamBreak transferEndBB;

        public final HardwareMap hardwareMap;
        public final LynxModuleInitializer modules;
        public final OctoQuad octoQuad;

        //Things that could move get inited here. This only should be called when movment is allowable
        private Container(HardwareMap hardwareMap, LynxModuleInitializer modules, OctoQuad octoQuad) {
            this.octoQuad = octoQuad;
            this.hardwareMap = hardwareMap;
            this.modules = modules;

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
            leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
            rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intakeDrive = child.createDcMotor(intakeDriveID);
            intakeDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            transferDrive = child.createDcMotor(moverDriveID);

            launchDrive = child.createDcMotor(launchDriveID);
            launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launchDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            launchDrive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, launcherCoefficients);
            launchDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launchDrive.setVelocity(0);
            launchDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            turretServo = child.createServo(turretServoID);
            turretServo.setPosition(turretCenter);

            rampServo = child.createServo(kickerServoID);
            rampServo.setPosition(MainBotHardware.rampDownPos);

            intakeBB = new BeamBreak(child.createDigitalChannel(intakeBBID));
            transferMiddleBB = new BeamBreak(child.createDigitalChannel(transferMiddleBBID));
            transferEndBB = new BeamBreak(child.createDigitalChannel(transferEndBBID));

            //This is kind of jank, but it works.
            lazyImu = () -> {
                throw new RuntimeException("We don't support the lazy IMU");
            };

            for(Map.Entry<String, VoltageSensor> entry : hardwareMap.voltageSensor.entrySet()) {
                hardwareMap.voltageSensor.remove(entry.getKey());
            }

            for(HardwareDevice device : hardwareMap.getAll(VoltageSensor.class)) {
                String name = String.valueOf(hardwareMap.getNamesOf(device));

                RobotLog.ii("MainBotHardware", "Removing voltage sensor called : \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
                hardwareMap.remove(name, device);
                device.close();
            }

            try {
                voltageSensor = new ProxyVoltageSensor(child.createAnalogInput(0), new LynxVoltageSensor(hardwareMap.appContext, child.module), resistorMultiplier);
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }
            hardwareMap.put("ProxyVoltageSensor", voltageSensor);
            hardwareMap.voltageSensor.put("ProxyVoltageSensor", voltageSensor);
        }

        public void turretToHeading(double heading) {
            if(Math.abs(heading) > turretMaxHeading) {
                heading = Math.signum(heading) * turretMaxHeading;
            }
            //Multiply by this to convert from radians to servo pos
            double turretRatio = 77.0/29 * 15/48 / Math.toRadians(axonRange);
            double pos = turretRatio * heading + turretCenter;
            //RobotLog.ii("MainBotHardware", "Turret: " + pos);
            getContainer().turretServo.setPosition(pos);
        }


        public void setOctoQuadPose(Pose2d pose) {
            setOctoQuadPose(pose.position.x, pose.position.y, (float) pose.heading.toDouble());
        }

        public void setOctoQuadPose(double x, double y, float heading) {
            octoQuad.setLocalizerPose((int) (x * 25.4), (int) (y * 25.4), heading);
        }

        private final int octoquadMaxRetryAttempts = 10;

        public OctoQuad.LocalizerDataBlock readOctoQuadLocalizerData(Supplier<Boolean> isStopRequested) {
            int failedOctoquadReadAttempts = 0;
            OctoQuad.LocalizerDataBlock block = null;
            while((block == null || !block.crcOk)) {
                if(isStopRequested == null) {
                    if(failedOctoquadReadAttempts >= octoquadMaxRetryAttempts) {
                        block = null;
                        break;
                    }
                    failedOctoquadReadAttempts++;
                } else if(isStopRequested.get()) {
                    block = null;
                    break;
                }
                block = octoQuad.readLocalizerData();
            }

            return block;
        }

        public void tickBBs() {
            Container hardware = getContainer();
            hardware.intakeBB.tick();
            hardware.transferMiddleBB.tick();
            hardware.transferEndBB.tick();
        }

        public void restIntakeTransfer() {
            Container hardware = getContainer();
            hardware.transferDrive.setPower(0);
            hardware.intakeDrive.setPower(0);
            hardware.rampServo.setPosition(MainBotHardware.rampDownPos);
        }
    }

    public static Pose2d makeOctoQuadPose(OctoQuad.LocalizerDataBlock block) {
        return new Pose2d(block.posX_mm / 25.4, block.posY_mm / 25.4, block.heading_rad);
    }

    private final OctoQuad octoQuad;
    private final HardwareMap hardwareMap;
    private final LynxModuleInitializer modules;

    public static MainBotHardware getOrCreate(HardwareMap hardwareMap, boolean resetOctoQuad) {
        MainBotHardware ts = SingletonHelper.getOrCreate(MainBotHardware.class, () -> new MainBotHardware(hardwareMap, resetOctoQuad));
        if(ts.modules != LynxModuleInitializer.getOrCreate(hardwareMap)) {
//            ts = new MainBotHardware(hardwareMap);
            SingletonHelper.clear(MainBotHardware.class);
        }
        return SingletonHelper.getOrCreate(MainBotHardware.class, () -> new MainBotHardware(hardwareMap, resetOctoQuad));
    }

    public static MainBotHardware getOrCreate(HardwareMap hardwareMap) {
        return getOrCreate(hardwareMap, true);
    }

    public static void clearInstance() {
        SingletonHelper.clear(MainBotHardware.class);
    }

    private MainBotHardware(HardwareMap hardwareMap, boolean resetOctoQuad) {
        this.hardwareMap = hardwareMap;
        RobotLog.ii("MainBotHardware", "Initializing mainbothardware");

        modules = LynxModuleInitializer.getOrCreate(hardwareMap);
        if(modules.childModule == null) {
            throw new RuntimeException("No child module found. A child module is required for MainBotHardware");
        }

        LynxModuleInitializer.LynxModuleControllers parent = modules.parentModule;
        LynxModuleInitializer.LynxModuleControllers child = modules.childModule;

        //OctoQuad is initialized first so nothing jolts while it is calibrating
        LynxI2cDeviceSynch synch = LynxFirmwareVersionManager.createLynxI2cDeviceSynch(hardwareMap.appContext, parent.module, 0);
        synch.setUserConfiguredName("octoquad");
        octoQuad = new OctoQuadImpl(synch, true);
        if(resetOctoQuad) {
            octoQuad.resetEverything();
            octoQuad.setCachingMode(OctoQuad.CachingMode.MANUAL);
            octoQuad.setAllVelocitySampleIntervals(10);
            octoQuad.setSingleEncoderDirection(1, OctoQuad.EncoderDirection.REVERSE);
            octoQuad.setAllLocalizerParameters(
                    xEncoderIndex,
                    yEncoderIndex,
                    encoderTicksPerMM,
                    encoderTicksPerMM,
                    15,
                    -190,
                    1.0156472633360971f,
                    25
            );

            octoQuad.resetLocalizerAndCalibrateIMU();
            while(octoQuad.getLocalizerStatus() != OctoQuad.LocalizerStatus.RUNNING) {
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
