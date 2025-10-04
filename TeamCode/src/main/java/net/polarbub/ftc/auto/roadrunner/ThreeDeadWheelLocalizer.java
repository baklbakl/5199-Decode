package net.polarbub.ftc.auto.roadrunner;

import android.util.Log;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import net.polarbub.ftc.MainBotConfig;
import net.polarbub.ftc.MainBotHardware;
import net.polarbub.ftc.auto.roadrunner.messages.ThreeDeadWheelInputsMessage;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = -1889; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1889; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = -1182.808938012382; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;

    private final LazyImu imu;

    public ThreeDeadWheelLocalizer(MainBotHardware hardware, double inPerTick, LazyImu imu) {
//        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, MainBotConfig.paraEncoder0ID)));
//        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class,MainBotConfig.paraEncoder1ID)));
//        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, MainBotConfig.perpEncoderID)));
        par0 = hardware.parallelEncoder0;
        par1 = hardware.parallelEncoder1;
        perp = hardware.perpendicularEncoder;

        this.inPerTick = inPerTick;

        this.imu = imu;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);
    }

//    private int startPar0 = 0;
//    private int startPar1 = 0;
//    private double startIMU = 0;

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

//            startPar0 = par0PosVel.position;
//            startPar1 = par1PosVel.position;
//            startIMU = imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//            Log.i("CUSTOM_LOCALIZATION", "par0 start: " + startPar0);
//            Log.i("CUSTOM_LOCALIZATION", "par1 start: " + startPar1);
//            Log.i("CUSTOM_LOCALIZATION", "imu start: " + startIMU);

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

//        int truePar0 = par0PosVel.position - startPar0;
//        int truePar1 = par1PosVel.position - startPar1;
//        if(!(par0PosDelta == 0 && par1PosDelta == 0)) {
//            Log.i("CUSTOM_LOCALIZATION", "par0: " + truePar0);
//            Log.i("CUSTOM_LOCALIZATION", "par1: " + truePar1);
//            Log.i("CUSTOM_LOCALIZATION", "heading: " + Math.toDegrees((truePar0 - truePar1) / (PARAMS.par0YTicks - PARAMS.par1YTicks)));
//            Log.i("CUSTOM_LOCALIZATION", "imu: " + (imu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startIMU) + "\n");
//        }

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}
