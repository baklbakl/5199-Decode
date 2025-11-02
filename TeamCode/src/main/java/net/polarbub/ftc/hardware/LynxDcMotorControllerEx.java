package net.polarbub.ftc.hardware;

import android.content.Context;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.*;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.util.LastKnown;
import com.qualcomm.robotcore.util.RobotLog;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Arrays;

public class LynxDcMotorControllerEx extends LynxDcMotorController {
    private LastKnown<Double>[] lastKnownTargetVelocities = new LastKnown[LynxConstants.NUMBER_OF_MOTORS];

    public LynxDcMotorControllerEx(Context context, LynxModule module) throws RobotCoreException, InterruptedException {
        super(context, module);

        for(int i = 0; i < LynxConstants.NUMBER_OF_MOTORS; i++) {
            lastKnownTargetVelocities[i] = new LastKnown<>();
        }
    }



    private void validateMotor(int motor) {
        try {
            Method m = LynxDcMotorController.class.getDeclaredMethod("validateMotor", int.class);
            m.setAccessible(true);
            m.invoke(this, motor);
        } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
            throw new RuntimeException(e);
        }
    }

    //Add caching for motor velocity
    @Override public synchronized void setMotorVelocity(int motor, double ticksPerSecond) {
        validateMotor(motor);
        int internalMotor = motor - apiMotorFirst;
        if(lastKnownTargetVelocities[internalMotor].updateValue(ticksPerSecond)) {
            super.setMotorVelocity(motor, ticksPerSecond);
        }
    }

    public synchronized double getTargetMotorVelocity(int motor)
    {
        validateMotor(motor);
        int internalMotor = motor - apiMotorFirst;
        Double value = lastKnownTargetVelocities[internalMotor].getValue();
        if(value == null) {
            DcMotor.RunMode mode = internalGetPublicMotorMode(motor);
            try {
                switch (mode)
                {
                    case RUN_TO_POSITION:
                    case RUN_USING_ENCODER:
                    {
                        LynxGetMotorTargetVelocityCommand command = new LynxGetMotorTargetVelocityCommand(this.getModule(), motor);
                        LynxGetMotorTargetVelocityResponse response = command.sendReceive();

                        // Scale relative to the *current* max speed
                        int iVelocity = response.getVelocity();
                        double velocity = iVelocity;
                        lastKnownTargetVelocities[motor].setValue(velocity);
                        if (DEBUG) RobotLog.vv(TAG, "getTargetMotorVelocity: mod=%d motor=%d velocity=%d", getModuleAddress(), motor, iVelocity);
                        return velocity;
                    }
                    case RUN_WITHOUT_ENCODER:
                    default:
                        throw new RuntimeException("Invalid motor mode \"" + mode + "\" for getTargetVelocity");
                }

            } catch (InterruptedException | RuntimeException | LynxNackException e)
            {
                handleException(e);
                return 0;
            }

        } else {
            return value;
        }

    }
}
