package org.firstinspires.ftc.teamcode.misc;

import java.util.function.BooleanSupplier;

public class SinglePressButton {

    private final BooleanSupplier booleanSupplier;
    private boolean lastButtonState = false;

    public SinglePressButton(BooleanSupplier booleanSupplier) {
        this.booleanSupplier = booleanSupplier;
    }


    public boolean get() {
        boolean currentButtonState = booleanSupplier.getAsBoolean();
        boolean trigger = ((currentButtonState != lastButtonState) && currentButtonState);
        lastButtonState = currentButtonState;
        return trigger;
    }
}
