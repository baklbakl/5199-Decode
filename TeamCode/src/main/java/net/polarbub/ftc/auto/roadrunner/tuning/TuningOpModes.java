package net.polarbub.ftc.auto.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import net.polarbub.ftc.auto.tuning.ManualStrafeFeedforwardTuner;
import net.polarbub.ftc.hardware.LynxModuleInitializer;
import net.polarbub.ftc.hardware.MainBotHardware;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import net.polarbub.ftc.auto.roadrunner.MecanumDrive;
import net.polarbub.ftc.auto.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

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
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf = hardwareMap -> {
            MainBotHardware hardware = MainBotHardware.getOrCreate(hardwareMap);
            MecanumDrive md = new MecanumDrive(hardware, new Pose2d(0, 0, 0));

//            List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
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
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
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
                    md.voltageSensor,
                    //It would be nice if this were a AsyncFeedForward, but we would have to modify RR which isn't too fun to do
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            (MecanumDrive.PARAMS.kASpeedUp + MecanumDrive.PARAMS.kASlowDown) / 2 / MecanumDrive.PARAMS.inPerTick),
                    0
            );
        };


        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
//        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        manager.register(metaForClass(ManualStrafeFeedforwardTuner.class), new ManualStrafeFeedforwardTuner(dvf, MecanumDrive.feedForwardSupplier));
        manager.register(metaForClass(net.polarbub.ftc.auto.tuning.ManualFeedforwardTuner.class), new net.polarbub.ftc.auto.tuning.ManualFeedforwardTuner(dvf, MecanumDrive.feedForwardSupplier));
        manager.register(metaForClass(ManualStrafeFeedbackTuner.class), ManualStrafeFeedbackTuner.class);
        manager.register(metaForClass(TurnTest.class), TurnTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
//                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class,
                    MecanumDrive.class,
                    ThreeDeadWheelLocalizer.class,

                    ManualStrafeFeedforwardTuner.class,
                    ManualStrafeFeedbackTuner.class,
                    net.polarbub.ftc.auto.tuning.ManualFeedforwardTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
