package org.ducktapeandaprayer.ftc.lib;

import android.content.Context;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;

public class OnCreateHandlers {
    private static WeakReference<FtcEventLoop> eventLoop = null;
    private static WeakReference<FtcEventLoopHandler> eventLoopHandler = null;
    private static WeakReference<OpModeManagerImpl> opModeManagerRef = null;

    @org.firstinspires.ftc.ftccommon.external.OnCreate
    public static void onCreate(Context context) {
        //This seems as a good a time as any to register the warning source
        RobotLog.registerGlobalWarningSource(new TeamCodeWarningSource());
    }

    @org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
    public static void collectEventLoop(Context context, FtcEventLoop loop) {
        eventLoop = new WeakReference<>(loop);

        if(opModeManagerRef != null && opModeManagerRef.get() != null) {
            opModeManagerRef.get().unregisterListener(OpModeNotificationListener.instance);
        }

        OpModeManagerImpl opModeManager = loop.getOpModeManager();
        opModeManager.registerListener(OpModeNotificationListener.instance);
        opModeManagerRef = new WeakReference<>(opModeManager);

        try {
            Field loopHandlerField = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            loopHandlerField.setAccessible(true);
            FtcEventLoopHandler loopHandler = (FtcEventLoopHandler) loopHandlerField.get(eventLoop.get());

            //Change the robot battery level send interval to be every 750ms from once per three seconds, but it is a protected field, so use reflection
            Field robotBatteryIntervalField = loopHandler.getClass().getDeclaredField("robotBatteryInterval");
            robotBatteryIntervalField.setAccessible(true);
            robotBatteryIntervalField.set(loopHandler, 0.2);

            eventLoopHandler = new WeakReference<>(loopHandler);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("collectEventLoop reflection oopsy", e);
        }
    }

    public static WeakReference<FtcEventLoop> getEventLoop() {
        return eventLoop;
    }

    public static WeakReference<EventLoopManager> getEventLoopManger() {
        return new WeakReference<>(eventLoopHandler.get().getEventLoopManager());
    }
}
