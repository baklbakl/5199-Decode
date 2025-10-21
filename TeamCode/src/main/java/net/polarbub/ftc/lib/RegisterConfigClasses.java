package net.polarbub.ftc.lib;

import android.content.Context;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.SplitAuto;
import net.polarbub.ftc.tele.MainTeleop;
import net.polarbub.ftc.tele.SensorReader;
import net.polarbub.ftc.tele.Test;
import org.firstinspires.ftc.ftccommon.external.OnCreate;

public class RegisterConfigClasses {
    private static final Class<?>[] classesToAdd = {
            MainBotConfig.MoveConfig.class,
            SplitAuto.class,
            SensorReader.class,
            MainTeleop.class,
            Test.class
    };

    @OnCreate
    public static void start(Context context) {
        FtcDashboard.suppressOpMode();
        FtcDashboard.start(context);
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.withConfigRoot(customVariable -> {
            try {
                for(Class<?> clazz : classesToAdd) {
                    customVariable.putVariable(clazz.getSimpleName(), ReflectionConfig.createVariableFromClass(clazz));
                }
                dash.updateConfig();
            } catch (NoClassDefFoundError ignored) {
                // dash is unable to access many classes and reporting every instance
                // only clutters the logs
            }
        });
    }
}
