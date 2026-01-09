package org.ducktapeandaprayer.ftc.dashConfig;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

public class ButtonValueProvider implements ValueProvider<Boolean> {
    private final Runnable onPress;
    public ButtonValueProvider(Runnable onPress) {
        this.onPress = onPress;
    }

    private boolean resultToSend = false;

    @Override
    public Boolean get() {
        return resultToSend;
    }

    @Override
    public void set(Boolean value) {
        if(value) {
            this.onPress.run();
        }

        resultToSend = true;
        FtcDashboard.getInstance().updateConfig();
        resultToSend = false;
    }
}
