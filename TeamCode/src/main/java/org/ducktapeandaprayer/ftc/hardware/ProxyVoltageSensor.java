package org.ducktapeandaprayer.ftc.hardware;

import android.content.Context;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//FIX: Make proxyVoltageSensor fall back if it hasn't been ticked in a while
public class ProxyVoltageSensor implements VoltageSensor {

    private final double scalingFactor;
    private final AnalogInput input;
    private final double fallbackVoltageSensorTimeout = 1.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final VoltageSensor fallbackVoltageSensor;

    public ProxyVoltageSensor(AnalogInput analogInput, VoltageSensor fallbackVoltageSensor, double scalingFactor) {
        this.input = analogInput;
        this.fallbackVoltageSensor = fallbackVoltageSensor;
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
        if(timer.seconds() > fallbackVoltageSensorTimeout) {
            //RobotLog.ii("ProxyVoltageSensor","Read Fallback");
            return fallbackVoltageSensor.getVoltage();
        } else {
            double proxyVoltage = input.getVoltage() * scalingFactor;
            //Voltage sensor came unplugged. Fall back.
            if(proxyVoltage < 3) {
                return fallbackVoltageSensor.getVoltage();
            };
            //RobotLog.ii("ProxyVoltageSensor","Read bulk");
            return proxyVoltage;
        }
    }

    public void periodic() {
        //RobotLog.ii("ProxyVoltageSensor","Tick");
        timer.reset();
    }

}
