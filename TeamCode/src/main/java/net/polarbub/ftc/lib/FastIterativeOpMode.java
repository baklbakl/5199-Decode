package net.polarbub.ftc.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//ADD: Some way of hot swapping opmodes
public abstract class FastIterativeOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        init();
        init_internal();

        while (!this.isStopRequested() && !this.isStarted()) {
            init_loop_internal();
        }
//        waitForStart();

        if(opModeIsActive()) {
            start_internal();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                loop_internal();
            }
        }
        stop_internal();
    }

    protected abstract void stop_internal();

    protected abstract void start_internal();

    protected abstract void loop_internal();

    protected abstract void init_loop_internal();

    protected abstract void init_internal();
}
