package org.firstinspires.ftc.teamcode.misc

import java.util.function.BooleanSupplier

class ButtonSwitch(private val booleanSupplier: BooleanSupplier) {
    private var lastButtonState = false
    private var trigger = false
    fun get(): Boolean {
        val currentButtonState = booleanSupplier.asBoolean
        trigger = (currentButtonState != lastButtonState && currentButtonState) != trigger
        lastButtonState = currentButtonState
        return trigger
    }
}