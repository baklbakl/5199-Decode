package org.ducktapeandaprayer.ftc.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.RobotLog;
import org.ducktapeandaprayer.ftc.hardware.*;
import org.ducktapeandaprayer.ftc.lib.ThreadEx;
import org.ducktapeandaprayer.ftc.lib.ThreadedOpMode;
import org.ducktapeandaprayer.ftc.lib.optiondriver.BooleanOption;
import org.ducktapeandaprayer.ftc.lib.optiondriver.OptionDriver;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import java.util.function.Supplier;

public class RobotImplThread extends ThreadEx implements RobotInterface {
    private final MainBotHardware hardware;
    private final MainBotHardware.Container container;
    private final AutoShoot autoShoot;
    private final AutoIntake autoIntake;
    private final AutoRanging autoRanging;
    private final Supplier<Pose2d> poseSupplier;

    private volatile boolean requestLaunchingStart = false;
    private volatile boolean launching = false;
    private volatile boolean full = false;
    private final boolean runAutoRanging;
    private final AtomicReference<Double> requestLauncherSpeed = new AtomicReference<>(Double.NaN);
    private final AtomicReference<Boolean> requestIntake = new AtomicReference<>(null);
    private final AtomicReference<Double> voltage = new AtomicReference<>(Double.NaN);

    private boolean intaking = false;
    private boolean autoRangingDebugInfo = true;

    public RobotImplThread(MainBotHardware hardware) {
        this(hardware, false, null, null);
    }

    public RobotImplThread(MainBotHardware hardware, ThreadedOpMode opMode, Supplier<Pose2d> poseSupplier) {
        this(hardware, true, opMode, poseSupplier);
    }

    private RobotImplThread(MainBotHardware hardware, boolean runAutoRanging, ThreadedOpMode opMode, Supplier<Pose2d> poseSupplier) {
        this.hardware = hardware;
        this.poseSupplier = poseSupplier;
        this.runAutoRanging = runAutoRanging;

        this.container = hardware.getContainer();
        container.modules.childModule.module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        this.autoShoot = new AutoShoot(hardware);
        this.autoIntake = new AutoIntake(hardware);
        if(runAutoRanging) {
            this.autoRanging = new AutoRanging(hardware, opMode::invertIfBlue);
            try {
                opMode.getOptionDriver().add(new BooleanOption("Auto ranging debug info", "dpad_right", opMode.gamepad1, this.getClass().getField("autoRangingDebugInfo"), this));
            } catch (NoSuchFieldException e) {
                throw new RuntimeException(e);
            }
            this.autoRanging.setEnableDebugInfo(true);
            //These are only fine to run in init because we are setting them to true
            autoRanging.setTurretTracking(true);
            autoRanging.setLauncherRunning(false);
        } else {
            this.autoRanging = null;
        }

    }

    @Override
    public void run() {
        if(autoRanging != null) {
            autoRanging.setEnableDebugInfo(autoRangingDebugInfo);
        }
        super.run();
    }

    @Override
    public void tick() {
        container.modules.childModule.module.clearBulkCache();
        container.tickBBs();
        voltage.set(container.voltageSensor.getVoltage());

        if(runAutoRanging) {
            String out = autoRanging.tick(poseSupplier.get());
            if(!out.isEmpty()) {
                telemetry.append(out);
            }
        }

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

        full = container.transferEndBB.getState() == BeamBreak.State.FULL && container.transferMiddleBB.getState() == BeamBreak.State.FULL && container.intakeBB.getState() == BeamBreak.State.FULL;

        synchronized (requestLauncherSpeed) {
            double speed = requestLauncherSpeed.get();
            if(!Double.isNaN(speed)) {
                if(runAutoRanging) {
                    autoRanging.setLauncherRunning(speed != 0);
                } else {
                    container.launchDrive.setVelocity(speed);
                    requestLauncherSpeed.set(Double.NaN);
                }
            }
        }

        synchronized (requestIntake) {
            Boolean request = requestIntake.get();
            if(request != null) {
                if(request) {
                    autoIntake.startIntaking();
                    intaking = true;
                } else if(intaking) {
                    container.restIntakeTransfer();
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

    public boolean isFull() {
        return full;
    }
}
