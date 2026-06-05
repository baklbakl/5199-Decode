package org.ducktapeandaprayer.ftc.hardware;

import android.content.Context;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;

//FIX: Make proxyVoltageSensor fall back if it hasn't been ticked in a while
public class ProxyVoltageSensor implements VoltageSensor {

    private final double scalingFactor;
    private final AnalogInput input;

    public ProxyVoltageSensor(AnalogInput analogInput, double scalingFactor) {
        this.input = analogInput;
        this.scalingFactor = scalingFactor;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Voltage Sensor from scaled analog input";
    }

    @Override
    public String getConnectionInfo() {
        return "Scaled From " + input.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }

    @Override
    public double getVoltage() {
        return input.getVoltage() * scalingFactor;
    }
}
