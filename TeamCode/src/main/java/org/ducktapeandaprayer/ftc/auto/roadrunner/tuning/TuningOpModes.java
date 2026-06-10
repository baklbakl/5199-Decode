package org.ducktapeandaprayer.ftc.auto.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.ducktapeandaprayer.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.ducktapeandaprayer.ftc.auto.roadrunner.MecanumDrive;
import org.ducktapeandaprayer.ftc.auto.roadrunner.ThreeDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class TuningOpModes {
//    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;

    public static final String GROUP = "RoadRunner";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        OpModeMeta meta = new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .setSource(OpModeMeta.Source.ANDROID_STUDIO)
                .build();

        return meta;
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf = hardwareMap -> {
            MainBotHardware hardware = MainBotHardware.getOrCreate(hardwareMap);
            MecanumDrive md = new MecanumDrive(hardware, new Pose2d(0, 0, 0), hardware.voltageSensor::getVoltage);

            List<EncoderGroup> encoderGroups = new ArrayList<>();
            List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
            if (!(md.localizer instanceof ThreeDeadWheelLocalizer)) {
                throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
            }

            ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;

            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    Arrays.asList(hardware.modules.parentModule.module, hardware.modules.childModule.module),
                    Arrays.asList(dl.par0, dl.par1, dl.perp)
            ));
            parEncs.add(new EncoderRef(0, 0));
            parEncs.add(new EncoderRef(0, 1));
            perpEncs.add(new EncoderRef(0, 2));

            return new DriveView(
                    DriveType.MECANUM,
                    1,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    MecanumDrive.PARAMS.maxAngVel,
                    MecanumDrive.PARAMS.maxAngAccel,
                    () -> MecanumDrive.PARAMS.trackWidth,
                    () -> MecanumDrive.PARAMS.lateralMultiplier,
                    encoderGroups,
                    Arrays.asList(
                            md.leftFront,
                            md.leftBack
                    ),
                    Arrays.asList(
                            md.rightFront,
                            md.rightBack
                    ),
                    Collections.emptyList() /*leftEncs*/,
                    Collections.emptyList() /*rightEncs*/,
                    parEncs,
                    perpEncs,
                    md.hardware.lazyImu,
                    (view) -> {
                        dl.update();
//                        RobotLog.ii("TuningOpModes", String.valueOf(dl.getPoseVelocity().angVel));
//                        RobotLog.ii("TuningOpModes", String.valueOf(Math.toDegrees(dl.getPoseVelocity().angVel)));
                        return new DualNum<>(Arrays.asList(dl.getPose().heading.toDouble(), dl.getPoseVelocity().angVel));

                    },
                    hardware.voltageSensor,
                    MecanumDrive.feedForwardFactory,
                    0
            );
        };


        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(ManualFeedforwardStrafeTuner.class), new ManualFeedforwardStrafeTuner(dvf));
        manager.register(metaForClass(ManualFeedforwardAngularTuner.class), new ManualFeedforwardAngularTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        //Using the .class ones erases the group name from the meta for some reason
        manager.register(metaForClass(ManualStrafeFeedbackTuner.class), new ManualStrafeFeedbackTuner());
        manager.register(metaForClass(ManualFeedbackTuner.class), new ManualFeedbackTuner());
        manager.register(metaForClass(SplineTest.class), new SplineTest());
        manager.register(metaForClass(TurnTest.class), new TurnTest());
        manager.register(metaForClass(LocalizationTest.class), new LocalizationTest());

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardStrafeTuner.class,
                    ManualFeedforwardAngularTuner.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class,
                    ManualStrafeFeedbackTuner.class,
                    MecanumDrive.class,
                    ThreeDeadWheelLocalizer.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
