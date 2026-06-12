package org.ducktapeandaprayer.ftc.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.hardware.AutoIntake;
import org.ducktapeandaprayer.ftc.hardware.AutoShoot;
import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;

import java.util.concurrent.atomic.AtomicReference;

public class RobotImplThread extends ThreadEx implements RobotInterface {
    private final MainBotHardware hardware;
    private final MainBotHardware.Container container;
    private final AutoShoot autoShoot;
    private final AutoIntake autoIntake;

    private volatile boolean requestLaunchingStart = false;
    private volatile boolean launching = false;
    private final AtomicReference<Double> requestLauncherSpeed = new AtomicReference<>(Double.NaN);
    private final AtomicReference<Boolean> requestIntake = new AtomicReference<>(null);
    private final AtomicReference<Double> voltage = new AtomicReference<>(Double.NaN);

    private boolean intaking = false;

    public RobotImplThread(MainBotHardware hardware) {
        this.hardware = hardware;

        this.container = hardware.getContainer();
        container.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        this.autoShoot = new AutoShoot(hardware);
        this.autoIntake = new AutoIntake(hardware);
    }

    @Override
    public void run() {
        super.run();
    }

    @Override
    public void tick() {
        container.modules.childModule.module.clearBulkCache();
        hardware.tickBBs();

        voltage.set(container.voltageSensor.getVoltage());

        if(requestLaunchingStart) {
            if(launching) {
                throw new RuntimeException("Can't start lauching while launching, but stupid because we didn't catch it outside this thread");
            }

            autoShoot.startShooting();
            launching = true;
            requestLaunchingStart = false;
            //ADD: Check to make sure we have some artifacts before starting launching. Also add a launching timeout
        }

        if(launching) {
            launching = !autoShoot.tick();
            //We just finished
            if(!launching) {
                autoShoot.stopShooting();
            }

        //We must be intaking or idling in which case then we should stop the intake and transfer when they are full
        } else if(intaking) {
            intaking = !autoIntake.tick();
            //No cleanup needed here. When intaking is done, everything is stopped
        }

        synchronized (requestLauncherSpeed) {
            double speed = requestLauncherSpeed.get();
            if(!Double.isNaN(speed)) {
                container.launchDrive.setVelocity(speed);
                requestLauncherSpeed.set(Double.NaN);
            }
        }

        synchronized (requestIntake) {
            Boolean request = requestIntake.get();
            if(request != null) {
                if(request) {
                    autoIntake.startIntaking();
                    intaking = true;
                } else if(intaking) {
                    hardware.restIntakeTransfer();
                    intaking = false;
                }

                requestIntake.set(null);
            }
        }
    }

    @Override
    public void launch() {
        if(launching) {
            throw new RuntimeException("Can't start lauching while launching");
        }
        requestLaunchingStart = true;
    }

    @Override
    public boolean launching() {
        return launching || requestLaunchingStart;
    }

    @Override
    public void startLauncher() {
        requestLauncherSpeed.set(MainBotHardware.launcherPresetVelocity);
        RobotLog.ii("RobotThreadImpl", "Launcher request speed: " + requestLauncherSpeed.get());
    }

    @Override
    public void stopLauncher() {
        requestLauncherSpeed.set(0.0);
    }

    @Override
    public void intake() {
        if(launching) {
            throw new RuntimeException("Can't mess with intake while launching");
        }
        requestIntake.set(true);
    }

    @Override
    public void stopIntake() {
        if(launching) {
            throw new RuntimeException("Can't mess with intake while launching");
        }
        requestIntake.set(false);
    }

    public double getVoltage() {
        return voltage.get();
    }
}
