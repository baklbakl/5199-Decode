package net.polarbub.ftc.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.lib.FastIterativeOpMode;
import net.polarbub.ftc.lib.LoopTimeCalculator;
import net.polarbub.ftc.lib.variablewatcher.DoubleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.SingleVariableWatcher;
import net.polarbub.ftc.lib.variablewatcher.VariableWatcher;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

import static java.lang.Math.abs;

@TeleOp(name = "LauncherInternalControlTuner", group = MainBotConfig.opmodeGroupName)
public class LauncherInternalControlTuner extends FastIterativeOpMode {
    public static double kV = 0.0056;
    public static double kP = 0.05;

    private MainBotHardware hardware = null;
    private List<VariableWatcher> variablesWatched = new ArrayList<>();
    private MultipleTelemetry mt = null;
    private Telemetry dashboardTelemetry = null;
    private ElapsedTime runtime = null;
    private LoopTimeCalculator timer = null;

    private class ThreadSafeTracker<T> {
        private T obj;
        private T objOld;
        private final Object lock = new Object();
        private Consumer<T> func;

        public ThreadSafeTracker(T startingVal, Consumer<T> func) {
            this.func = func;
            this.obj = startingVal;
        }

        public void setNewVal(T t) {
            synchronized (lock) {
                obj = t;
            }
        }

        public T getCurrentVal() {
            synchronized (lock) {
                return obj;
            }
        }

        protected void tick() {
            //This funky system is to minimize the time that we need to lock.
            T oldOldObj = objOld;
            
            synchronized (lock) {
                if(obj != objOld) {
                    objOld = obj;
                }
            }

            if(oldOldObj != objOld) {
                func.accept(objOld);
            }
        }
    }

    private class ChildModuleControlThread extends Thread {
        private final List<ThreadSafeTracker<?>> trackers = new ArrayList<>();
        public final ThreadSafeTracker<Double> launcherVelocityTracker;
        public final ThreadSafeTracker<Float> triggerControlTracker;

        private final AtomicBoolean stop = new AtomicBoolean(false);
        private final LoopTimeCalculator timer = new LoopTimeCalculator();
        private final ElapsedTime runtime = new ElapsedTime();

        private double triggerControl = Double.NaN;

        private ArrayList<Pair<String, Object>> telemetryData = new ArrayList<>();
        private final Object telemetryLock = new Object();

        private double launcherTargetVelocity = 0;
        
        public ChildModuleControlThread() {
            launcherVelocityTracker = new ThreadSafeTracker<>(0.0, (targetVelocity) -> {
                launcherTargetVelocity = targetVelocity;
            });
            trackers.add(launcherVelocityTracker);

            triggerControlTracker = new ThreadSafeTracker<>(Float.NaN, (triggerControl) -> {
                this.triggerControl = triggerControl;
                if(!Float.isNaN(triggerControl)) {
                    hardware.launchDrive.setPower(triggerControl);
                } else {
                    hardware.launchDrive.setPower(0);
                }
            });
            trackers.add(triggerControlTracker);
        }

        public void stopAndJoin() {
            stop.set(true);

            //We do this instead of this.join() because the app will interrupt the thread calling join()
            while(this.isAlive()) {
                try {
                    this.join();
                } catch (InterruptedException e) {
                    RobotLog.ii("ChildModuleControlThread", "Interrupted while joining");
                }
            }
        }

        @Override
        public void run() {
            timer.reset();
            runtime.reset();

            while(!stop.get()) {
                hardware.modules.childModule.module.clearBulkCache();
                ArrayList<Pair<String, Object>> tempTelemetryData = new ArrayList<>();

                for(ThreadSafeTracker<?> t : trackers) {
                    t.tick();
                }

                //FIX: Calculate our own velocities from position
                double velocity = hardware.launchDrive.getVelocity();
                long thisNanos = runtime.nanoseconds();

                if(Double.isNaN(triggerControl)) {
                    if(launcherTargetVelocity == 0) {
                        hardware.launchDrive.setPower(0);
                    } else {
                        hardware.launchDrive.setPower(((launcherTargetVelocity - velocity) * kP + (launcherTargetVelocity) * kV) / hardware.modules.childModule.voltageSensor.getVoltage() );
                    }

                    if(launcherTargetVelocity != 0 && lastVelocityZero && startTime == -1) {
                        startTime = thisNanos;
                    }
                    lastVelocityZero = launcherTargetVelocity == 0;
                    tempTelemetryData.add(new ImmutablePair<>("Target Vel", launcherTargetVelocity));

                } else {
                    if(velocity != 0 && lastVelocityZero) {
                        startTime = thisNanos;
                    }
                    lastVelocityZero = velocity == 0;
                    tempTelemetryData.add(new ImmutablePair<>("Manual Control", null));
                }

                if(startTime != -1 && abs(velocity) >= abs(launcherTargetVelocity)) {
                    lastSpinUpTime = (thisNanos - startTime) / 1000000000.0d;
                    startTime = -1;
                }

                if(startTime == -1) {
                    tempTelemetryData.add(new ImmutablePair<>("Last spin up time", lastSpinUpTime));
                } else {
                    tempTelemetryData.add(new ImmutablePair<>("Spinning up ...", null));
                }

                tempTelemetryData.add(new ImmutablePair<>("Vel", hardware.launchDrive.getVelocity()));
                tempTelemetryData.add(new ImmutablePair<>("Child Thread Timing", timer.toString()));

                synchronized (telemetryLock) {
                    telemetryData = tempTelemetryData;
                }

                timer.tick();
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }

        public void addTelemetryData(Telemetry t) {
            synchronized (telemetryLock) {
                for(Pair<String, Object> item : telemetryData) {
                    if(item.getValue() != null) {
                        t.addData(item.getKey(), item.getValue());
                    } else {
                        t.addLine(item.getKey());
                    }
                }
            }
        }
    }

    private ChildModuleControlThread childThread = null;

    @Override
    protected void init_internal() {
        hardware = MainBotHardware.getOrCreate(hardwareMap);
        hardware.modules.enableBulkCaching(LynxModule.BulkCachingMode.MANUAL);

        childThread = new ChildModuleControlThread();

        mt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        runtime = new ElapsedTime();
        timer = new LoopTimeCalculator();

        try {
            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("dpad_up"), Boolean::equals, (dpad_up) -> {
                if(dpad_up) {
                    double currentVal = childThread.launcherVelocityTracker.getCurrentVal();
                    if(currentVal == MainBotHardware.teleOpLauncherNoVisionVelocity) {
                        currentVal = 0;
                    } else {
                        currentVal = MainBotHardware.teleOpLauncherNoVisionVelocity;
                    }

                    childThread.launcherVelocityTracker.setNewVal(currentVal);
                }
            }));

            variablesWatched.add(new SingleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("cross"), Boolean::equals, (cross) -> {
                if(cross) {
                    double currentVal = childThread.triggerControlTracker.getCurrentVal();
                    if(Double.isNaN(currentVal)) {
                        childThread.triggerControlTracker.setNewVal(gamepad1.left_trigger - gamepad1.right_trigger);
                    } else {
                        childThread.triggerControlTracker.setNewVal(Float.NaN);
                    }
                }
            }));

            variablesWatched.add(new DoubleVariableWatcher<>(gamepad1, gamepad1.getClass().getField("left_trigger"), gamepad1.getClass().getField("right_trigger"), Float::equals, (left_trigger, right_trigger) -> {
                double currentVal = childThread.triggerControlTracker.getCurrentVal();
                if (!Double.isNaN(currentVal)) {
                    childThread.triggerControlTracker.setNewVal(left_trigger - right_trigger);
                }
            }));
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    protected void init_loop_internal() {

    }

    @Override
    protected void start_internal() {
        runtime.reset();
        timer.reset();
        childThread.start();
    }

    private long startTime = -1;
    private boolean lastVelocityZero = false;
    private double lastSpinUpTime = 0;

    @Override
    protected void loop_internal() {
        //Check for changes of watched variables and run the handler if they did
        for(VariableWatcher variableWatcher : variablesWatched) {
            variableWatcher.tick();
        }

        timer.tick();
        mt.addData("Main thread timing",
                timer.toString()
        );

        childThread.addTelemetryData(mt);

        //These scale the graph window in the dashboard
        dashboardTelemetry.addData("Min", 0);
        dashboardTelemetry.addData("Max", 2000);

        mt.update();
    }

    @Override
    protected void stop_internal() {
        childThread.stopAndJoin();
    }
}
