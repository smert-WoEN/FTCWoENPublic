package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleConsumer;

import static java.lang.Math.abs;
import static java.lang.Math.min;
import static java.lang.Math.signum;

public class motorAccelerationLimiter {
    private final DoubleConsumer motorToControl;
    private final ElapsedTime looptime = new ElapsedTime();
    private final double maxAcceleration;
    private double currentVelocity = 0;

    public motorAccelerationLimiter(DoubleConsumer motorToControl, double maxAcceleration) {
        this.motorToControl = motorToControl;
        this.maxAcceleration = maxAcceleration;
        looptime.reset();
    }

    public void setVelocity(double requestedVelocity) {
        if (requestedVelocity == 0) {
            currentVelocity = 0;
        } else {
            currentVelocity += min(abs(requestedVelocity - currentVelocity), abs(looptime.seconds() * maxAcceleration)) * signum(requestedVelocity - currentVelocity);
        }
        looptime.reset();
        motorToControl.accept(currentVelocity);
    }
}
