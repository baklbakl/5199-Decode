package net.polarbub.ftc.dashConfig;

import android.content.Context;
import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.dashboard.config.variable.BasicVariable;
import com.acmerobotics.dashboard.config.variable.ConfigVariableDeserializer;
import com.acmerobotics.dashboard.config.variable.ConfigVariableSerializer;
import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageDeserializer;
import com.google.gson.GsonBuilder;
import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.auto.RobotImpl;
import net.polarbub.ftc.auto.SplitAuto;
import net.polarbub.ftc.auto.vision.AprilTagDetectionPipeline;
import net.polarbub.ftc.hardware.MainBotHardware;
import net.polarbub.ftc.tele.LauncherInternalControlTuner;
import net.polarbub.ftc.tele.MainTeleop;
import net.polarbub.ftc.tele.Test;
import org.apache.commons.lang3.tuple.MutablePair;
import org.firstinspires.ftc.ftccommon.external.OnCreate;

import java.lang.reflect.Field;

public class RegisterConfigClasses {
    private static final Class<?>[] classesToAdd = {
            MainBotConfig.class,
            MainBotHardware.class,
            SplitAuto.class,
            RobotImpl.class,
            Test.class,
            AprilTagDetectionPipeline.class,
            LauncherInternalControlTuner.class
    };

//    private static List<Double> testList = null;
//    private static List<MutablePair<MutablePair<Double, Boolean>, Integer>> testList2 = null;

    @OnCreate
    public static void start(Context context) {
        try {
            Field GSONField = DashboardCore.class.getField("GSON");
            GSONField.setAccessible(true);
            GSONField.set(null,
                    new GsonBuilder()
                        .registerTypeAdapter(Message.class, new MessageDeserializer())
                        .registerTypeAdapter(BasicVariable.class, new ConfigVariableSerializer())
                        .registerTypeAdapter(BasicVariable.class, new ConfigVariableDeserializer())
                        .registerTypeAdapter(CustomVariable.class, new ConfigVariableSerializer())
                        .registerTypeAdapter(CustomVariable.class, new ConfigVariableDeserializer())
                        .registerTypeAdapter(ListConfigVariable.class, new ConfigVariableSerializer())
                        .registerTypeAdapter(ListConfigVariable.class, new ConfigVariableDeserializer())
                        .registerTypeAdapter(CustomVariableEx.class, new ConfigVariableSerializer())
                        .registerTypeAdapter(CustomVariableEx.class, new ConfigVariableDeserializer())
                        .serializeNulls()
                        .create()
            );
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }

        FtcDashboard.start(context);
        FtcDashboard dash = FtcDashboard.getInstance();
        dash.withConfigRoot(customVariable -> {
            try {
                for(Class<?> clazz : classesToAdd) {
                    customVariable.putVariable(clazz.getSimpleName(), ReflectionConfig.createVariableFromClass(clazz));
                }

                CustomVariable teleOpVariable = ReflectionConfig.createVariableFromClass(MainTeleop.class);
                ListConfigVariable<MutablePair<Double, Double>> listConfigVar = new ListConfigVariable<>(
                        MainBotHardware.launchSpeedMapping,
                        () -> new MutablePair<>(0.0, 0.0),
                        new MutablePair<>(0.0, 0.0));
                teleOpVariable.putVariable("launchSpeedMapping", listConfigVar);
                customVariable.putVariable("MainTeleop", teleOpVariable);

//                testList = new ArrayList<>();
//                testList.add(1.0);
//                testList.add(2.0);
//                testList.add(3.0);
//
//                ListConfigVariable<Double> testVar = new ListConfigVariable<>(
//                        testList,
//                        () -> 0.0,
//                        0.0);
//                customVariable.putVariable("testList", testVar);
//
//                testList2 = new ArrayList<>();
//                testList2.add(new MutablePair<>(new MutablePair<>(0.0, false), 0));
//                testList2.add(new MutablePair<>(new MutablePair<>(1.0, true), 1));
//                testList2.add(new MutablePair<>(new MutablePair<>(2.0, false), 2));
//
//                ListConfigVariable<MutablePair<MutablePair<Double, Boolean>, Integer>> testVar2 = new ListConfigVariable<>(
//                        testList2,
//                        () -> new MutablePair<>(new MutablePair<>(0.0, false), 0),
//                        new MutablePair<>(new MutablePair<>(0.0, false), 0));
//                customVariable.putVariable("testVar2", testVar2);

                dash.updateConfig();
            } catch (NoClassDefFoundError ignored) {
                // dash is unable to access many classes and reporting every instance
                // only clutters the logs
            }
        });
    }
}
