package net.polarbub.ftc.lib;

import android.content.Context;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopBase;
import com.qualcomm.ftccommon.FtcEventLoopHandler;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.ref.WeakReference;
import java.lang.reflect.Field;

public class OnCreateHandlers {
    private static WeakReference<FtcEventLoop> eventLoop = null;
    private static WeakReference<EventLoopManager> eventLoopManger = null;

    @org.firstinspires.ftc.ftccommon.external.OnCreate
    public static void onCreate(Context context) {
        //This seems as a good a time as any to register the warning source
        RobotLog.registerGlobalWarningSource(new TeamCodeWarningSource());
    }

    @org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
    public static void collectEventLoop(Context context, FtcEventLoop loop) {
        eventLoop = new WeakReference<>(loop);

        try {
            Field loopHandlerField = FtcEventLoopBase.class.getDeclaredField("ftcEventLoopHandler");
            loopHandlerField.setAccessible(true);
            FtcEventLoopHandler loopHandler = (FtcEventLoopHandler) loopHandlerField.get(loop);
            eventLoopManger = new WeakReference<>(loopHandler.getEventLoopManager());
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException("MainBotHardware HardwareMap oopsy", e);
        }
    }

    public static WeakReference<FtcEventLoop> getEventLoop() {
        return eventLoop;
    }

    public static WeakReference<EventLoopManager> getEventLoopManger() {
        return eventLoopManger;
    }
}
