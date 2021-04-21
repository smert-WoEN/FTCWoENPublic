package org.firstinspires.ftc.teamcode.misc

import java.util.function.BooleanSupplier

class SinglePressButton(private val booleanSupplier: BooleanSupplier) {
    private var lastButtonState = false
    fun get(): Boolean {
        val currentButtonState = booleanSupplier.asBoolean
        val trigger = currentButtonState != lastButtonState && currentButtonState
        lastButtonState = currentButtonState
        return trigger
    }
}