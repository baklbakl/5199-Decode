package net.polarbub.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.lib.SingletonHelper;
import net.polarbub.ftc.lib.TeamCodeWarningSource;

import static net.polarbub.ftc.lib.OnCreateHandlers.*;

public class LynxModuleInitializer {
    public static LynxModuleInitializer getOrCreate(HardwareMap hardwareMap) {
        return SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap));
    }

    public final HardwareMap hardwareMap;
    public final LynxModuleControllers parentModule;
    public final LynxModuleControllers childModule;

    public class LynxModuleControllers {
        public final LynxModule module;
        public final LynxDcMotorControllerEx motorController;
        public final LynxServoController servoController;
        public final LynxAnalogInputController analogInputController;
        public final LynxVoltageSensor voltageSensor;

        public LynxModuleControllers(LynxModule module) {
            this.module = module;
            try {
                motorController = new LynxDcMotorControllerEx(hardwareMap.appContext, module);
                servoController = new LynxServoController(hardwareMap.appContext, module);
                analogInputController = new LynxAnalogInputController(hardwareMap.appContext, module);
                voltageSensor = new LynxVoltageSensor(hardwareMap.appContext, module);
            } catch (RobotCoreException | InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        public DcMotorExEx createDcMotor(int id) {
            DcMotorExEx motor;

            motor = new DcMotorExEx(motorController, id, DcMotorSimple.Direction.FORWARD, MotorConfigurationType.getMotorType(GoBILDA5202Series.class));
            motor.setPower(0);
            return motor;
        }

        public ServoImplEx createServo(int id) {
            return new ServoImplEx(servoController, id, ServoConfigurationType.getStandardServoType());
        }

        public AnalogInput createAnalogInput(int id) {
            return new AnalogInput(analogInputController, id);
        }

        public Encoder createEncoder(int id) {
            return new OverflowEncoder(new RawEncoder(createDcMotor(id)));
        }
    }

    private LynxModuleInitializer(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        // Empty the hardware map so no one tries to use it except for LynxModules and LynxUsbDevices because the
        // FTC Libs uses the hardware map the close those.
        for(HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            String name = String.valueOf(hardwareMap.getNamesOf(device));
            if(device instanceof LynxUsbDevice || device instanceof LynxEmbeddedIMU || device instanceof RevColorSensorV3) {
                RobotLog.ii("LynxModuleInitializer", "Leaving device called: \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
            } else {
                RobotLog.ii("LynxModuleInitializer", "Removing device called : \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
                hardwareMap.remove(name, device);
            }
        }

        try {
            UsbScanner scanner = UsbScanner.getOrCreate(hardwareMap);

            LynxUsbDevice usbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, scanner.lynxUsbSerialNumber, getEventLoopManger().get(), HardwareDeviceManager.createUsbManager());
            int parentModuleAddress;
            String errorString;
            if(Device.isRevControlHub()) {
                parentModuleAddress = LynxConstants.CH_EMBEDDED_MODULE_ADDRESS;
                errorString = "Could not find built in parent Lynx Module inside Control Hub (something has gone seriously wrong). Address " + parentModuleAddress;
            } else {
                parentModuleAddress = MainBotHardware.expansionHubAddress;
                errorString = "Could not find parent USB connected Expansion Hub with address " + parentModuleAddress + ". Check the connection and that the address is set correctly.";
            }

            try {
                parentModule = new LynxModuleControllers(usbDevice.getOrAddModule((new LynxModuleDescription.Builder(parentModuleAddress, true)).build()));
                parentModule.module.pingAndQueryKnownInterfacesAndEtc();
            } catch (RobotCoreException e) {
                throw new RuntimeException(errorString, e);
            }

            //Put the control hub back so that stopping when the opmode works right
            hardwareMap.put("Control Hub " + parentModule.module, parentModule.module);

            TeamCodeWarningSource.getInstance().clearGlobalWarning();

            LynxModuleControllers localChildModule = null;
            if(Device.isRevControlHub()) {
                try {
                    localChildModule = new LynxModuleControllers(
                            usbDevice.getOrAddModule((new LynxModuleDescription.Builder(1, false)).build())
                    );
                    localChildModule.module.pingAndQueryKnownInterfacesAndEtc();
                    //Put the control hub back so that stopping when the opmode works right
                    hardwareMap.put("Expansion Hub " + localChildModule.module.toString(), localChildModule.module);
                    
                } catch (RobotCoreException e) {
                    if(localChildModule != null) {
                        localChildModule.module.close();
                    }

                    localChildModule = null;
                    TeamCodeWarningSource.getInstance().setGlobalWarning(
                            "Could not find RS485 connected Expansion Hub with address "
                            + MainBotHardware.expansionHubAddress
                            + ". This may be expected if no Expansion Hub is present. Check that the hub is connected, " +
                            "powered up, and that the address is correct."
                    );
                }
            }
            childModule = localChildModule;
        } catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void enableBulkCaching(LynxModule.BulkCachingMode mode) {
        parentModule.module.setBulkCachingMode(mode);
        childModule.module.setBulkCachingMode(mode);
    }

    public void clearBulkCache() {
        parentModule.module.clearBulkCache();
        childModule.module.clearBulkCache();
    }
}
