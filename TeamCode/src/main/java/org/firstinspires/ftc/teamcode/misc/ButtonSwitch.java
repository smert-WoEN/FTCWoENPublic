package org.firstinspires.ftc.teamcode.misc;

import java.util.function.BooleanSupplier;

public class ButtonSwitch {

    private boolean lastButtonState = false;
    private boolean trigger = false;
    private final BooleanSupplier booleanSupplier;

    public ButtonSwitch(BooleanSupplier booleanSupplier) {
        this.booleanSupplier = booleanSupplier;
    }

    public boolean get() {
        boolean currentButtonState = booleanSupplier.getAsBoolean();
        trigger = ((currentButtonState != lastButtonState) && currentButtonState) != trigger;
        lastButtonState = currentButtonState;
        return trigger;
    }
}

