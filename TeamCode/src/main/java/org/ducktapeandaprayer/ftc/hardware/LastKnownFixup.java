package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.hardware.lynx.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.LastKnown;

import java.lang.reflect.Field;
import java.util.List;

public class LastKnownFixup {
    private static final double newLastKnownTime = 3000;
    private static final Field msFreshnessField;
    private static final Field moduleControllersField;

    static {
        try {
            msFreshnessField = LastKnown.class.getDeclaredField("msFreshness");
            msFreshnessField.setAccessible(true);
            moduleControllersField = LynxModule.class.getDeclaredField("controllers");
            moduleControllersField.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    private static void fixupLastKnown(LastKnown<?> lastKnown) {
        try {
            msFreshnessField.setDouble(lastKnown, newLastKnownTime);
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    public static void runFixup(HardwareMap hardwareMap) {
        //This fixes up all the last known values in the controllers to be a larger, more sensible number
        for(LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            try {
                List<LynxController> moduleControllers = (List<LynxController>) moduleControllersField.get(module);

                for(LynxController controller : moduleControllers) {
                    Field[] controllerFields = controller.getClass().getDeclaredFields();
                    for(Field controlerField : controllerFields) {
                        controlerField.setAccessible(true);
                        Object object = controlerField.get(controller);
                        if(object instanceof LastKnown[]) {
                            LastKnown<?>[] lastKnowns = (LastKnown<?>[]) object;
                            for(LastKnown<?> lastKnown : lastKnowns) {
                                fixupLastKnown(lastKnown);
                            }
                        } else if(object instanceof LastKnown) {
                            fixupLastKnown((LastKnown<?>) object);
                        }
                    }
                }
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
