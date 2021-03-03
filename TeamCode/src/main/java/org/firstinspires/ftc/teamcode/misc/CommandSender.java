package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

public class CommandSender {
    private final ElapsedTime lastCommandTimer = new ElapsedTime();
    private final DoubleConsumer doubleConsumer;
    private double timeout = 3000;
    private double lastValue = Double.NaN;

    public CommandSender(DoubleConsumer doubleConsumer) {
        this.doubleConsumer = doubleConsumer;
    }

    public CommandSender(DoubleConsumer doubleConsumer, double timeout_ms) {
        this.doubleConsumer = doubleConsumer;
        timeout = timeout_ms;
    }

    public void send(double value) {
        if (value != lastValue || lastCommandTimer.milliseconds() > timeout) {
            doubleConsumer.accept(value);
            lastValue = value;
            lastCommandTimer.reset();
        }
    }

}
