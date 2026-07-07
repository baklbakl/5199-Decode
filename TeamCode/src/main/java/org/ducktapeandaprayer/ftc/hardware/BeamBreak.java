package org.ducktapeandaprayer.ftc.hardware;

import com.qualcomm.robotcore.hardware.DigitalChannel;

public class BeamBreak {
    public static int requiredSamples = 7;

    private final DigitalChannel in;

    private int stateSamples = 0;
    private boolean state;

    public BeamBreak(DigitalChannel in) {
        this.in = in;
        this.in.setMode(DigitalChannel.Mode.INPUT);
    }

    public void tick() {
        //Inverted so blocked is true
        boolean newState = !in.getState();
        if(state == newState) {
            stateSamples++;
        } else {
            stateSamples = 0;
        }
        state = newState;
    }

    public boolean getRawState() {
        return state;
    }

    public enum State {
        UNCERTAIN,
        FULL,
        EMPTY
    }

    public State getState() {
        if(stateSamples >= requiredSamples) {
            return state ? State.FULL : State.EMPTY;
        }

        return State.UNCERTAIN;
    }
}
