package org.ducktapeandaprayer.ftc.hardware;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.*;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.lib.SingletonHelper;
import org.ducktapeandaprayer.ftc.lib.TeamCodeWarningSource;

import java.util.Map;

import static org.ducktapeandaprayer.ftc.lib.OnCreateHandlers.*;

public class LynxModuleInitializer {
    public static LynxModuleInitializer getOrCreate(HardwareMap hardwareMap, boolean useRS485) {
        LynxModuleInitializer ts = SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap, useRS485));

        if(!ts.scanner.equals(UsbScanner.getOrCreate(hardwareMap)) || ts.useRS485 != useRS485) {
            SingletonHelper.clear(LynxModuleInitializer.class);
        }

        return SingletonHelper.getOrCreate(LynxModuleInitializer.class, () -> new LynxModuleInitializer(hardwareMap, useRS485));
    }

    public static void clearInstance() {
        SingletonHelper.clear(LynxModuleInitializer.class);
    }

    public final HardwareMap hardwareMap;
    public final LynxModuleControllers parentModule;
    public final LynxModuleControllers childModule;
    private final UsbScanner scanner;
    private final boolean useRS485;

    public class LynxModuleControllers {
        public final LynxModule module;
        public final LynxDcMotorControllerEx motorController;
        public final LynxDigitalChannelController digitalController;
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
                digitalController = new LynxDigitalChannelController(hardwareMap.appContext, module);
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

        public DigitalChannel createDigitalChannel(int id) {
            return new DigitalChannelImpl(digitalController, id);
        }

        public Encoder createEncoder(int id) {
            return new OverflowEncoder(new RawEncoder(createDcMotor(id)));
        }
    }

    private LynxModuleInitializer(HardwareMap hardwareMap, boolean useRS485) {
        this.hardwareMap = hardwareMap;
        this.useRS485 = useRS485;

        RobotLog.ii("LynxModuleInitializer", "Initializing modules");
        // Empty the hardware map so no one tries to use it except for LynxModules and LynxUsbDevices because the
        // FTC Libs uses the hardware map the close them.
        for(HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            String name = String.valueOf(hardwareMap.getNamesOf(device));
            if(device instanceof LynxUsbDevice) {
                RobotLog.ii("LynxModuleInitializer", "Leaving device called: \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
            } else {
                RobotLog.ii("LynxModuleInitializer", "Removing device called : \"" + name + "\" Class: " + device.getClass().getCanonicalName() + " from hardwaremap");
                hardwareMap.remove(name, device);
            }
        }

        //Empty out device specific map
        for(HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : hardwareMap.allDeviceMappings) {
            for(Map.Entry<String, ? extends HardwareDevice> device : mapping.entrySet()) {
                mapping.remove(device.getKey());
            }
        }

        try {
            scanner = UsbScanner.getOrCreate(hardwareMap);

            LynxUsbDevice parentUsbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, scanner.lynxParentUsbSerialNumber, getEventLoopManger().get(), HardwareDeviceManager.createUsbManager());
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
                parentModule = new LynxModuleControllers(parentUsbDevice.getOrAddModule((new LynxModuleDescription.Builder(parentModuleAddress, true)).build()));
                parentModule.module.pingAndQueryKnownInterfacesAndEtc();
            } catch (RobotCoreException e) {
                throw new RuntimeException(errorString, e);
            }

            //Put the control hub back so that stopping when the opmode works right
            hardwareMap.put("Control Hub " + parentModule.module, parentModule.module);

            TeamCodeWarningSource.getInstance().clearGlobalWarning();

            LynxModuleControllers localChildModule = null;
            if(Device.isRevControlHub()) {
                LynxUsbDevice childUsbDevice = null;

                if(useRS485) {
                    childUsbDevice = parentUsbDevice;
                } else {
                    if(scanner.lynxChildUsbSerialNumber != null) {
                        childUsbDevice = LynxUsbDeviceImpl.findOrCreateAndArm(hardwareMap.appContext, scanner.lynxChildUsbSerialNumber, getEventLoopManger().get(), HardwareDeviceManager.createUsbManager());
                    } else {
                        TeamCodeWarningSource.getInstance().setGlobalWarning(
                                "Could not find USB Connected Expansion Hub. " +
                                        "This may be expected if no Expansion Hub is present. " +
                                        "Check that the hub is connected and powered up."
                        );
                    }
                }

                try {
                    if(childUsbDevice != null) {
                        localChildModule = new LynxModuleControllers(
                                childUsbDevice.getOrAddModule((new LynxModuleDescription.Builder(1, !useRS485)).build())
                        );

                        localChildModule.module.pingAndQueryKnownInterfacesAndEtc();
                        //Put the module in the hardware map that stopping when the opmode works right
                        hardwareMap.put("Expansion Hub " + localChildModule.module.toString(), localChildModule.module);
                        //Put the usb device in the hardware that disconnections and reconnections work right. This means that there will be issues if a new eh it installed without a power cycle.
                        if(childUsbDevice != parentUsbDevice) {
                            hardwareMap.put("Expansion Hub Portal Created" + childUsbDevice.toString(), childUsbDevice);
                        }

                    }
                } catch (RobotCoreException e) {
                    if(localChildModule != null) {
                        localChildModule.module.close();
                    }

                    //Only close the child usb device if it is distinct from the parent one. When we are not connected with RS485
                    if(!useRS485) {
                        childUsbDevice.close();
                    }

                    localChildModule = null;
                    TeamCodeWarningSource.getInstance().setGlobalWarning(
                            "Could not find Expansion Hub with address "
                                    + MainBotHardware.expansionHubAddress
                                    + " connected via " + (useRS485 ? "RS485. This may be expected if no Expansion Hub is present. "
                                    + "Check that the hub is connected, powered up, and that the address is correct"
                                    : "USB. This is strange because the USB device was found, but the lynx module itself was not. "
                                    + "Check the address of the module is correct")
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
