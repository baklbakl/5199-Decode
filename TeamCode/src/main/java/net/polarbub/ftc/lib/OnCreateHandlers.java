package net.polarbub.ftc.lib;

import android.content.Context;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.ref.WeakReference;

public class OnCreateHandlers {
    public static WeakReference<FtcEventLoop> eventLoop = null;

    @org.firstinspires.ftc.ftccommon.external.OnCreate
    public static void onCreate(Context context) {
        //This seems as a good a time as any to register the warning source
        RobotLog.registerGlobalWarningSource(new TeamCodeWarningSource());
    }

    @org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
    public static void collectEventLoop(Context context, FtcEventLoop loop) {
        eventLoop = new WeakReference<>(loop);
    }
}
