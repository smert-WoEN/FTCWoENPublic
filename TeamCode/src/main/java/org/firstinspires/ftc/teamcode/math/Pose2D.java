package org.firstinspires.ftc.teamcode.math;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap;

public class Pose2D extends Vector2D{
    public double heading;

    public Pose2D(double x, double y, double heading) {
        super(x, y);
        this.heading = angleWrap(heading);
    }

    public Pose2D() {
        this(0, 0, 0);
    }

    public Pose2D(Vector2D p, double heading) {
        this(p.x, p.y, angleWrap(heading));
    }

    public Pose2D plus(Pose2D p2) {
        return new Pose2D(x + p2.x, y + p2.y, heading + p2.heading);
    }


    public Pose2D times(Pose2D p2) {
        return new Pose2D(x * p2.x, y * p2.y, heading * p2.heading);
    }

    public Pose2D div(Pose2D p2) {
        return new Pose2D(x / p2.x, y / p2.y, heading / p2.heading);
    }

    public Pose2D div(double d) {
        return new Pose2D(x / d, y / d, heading / d);
    }

    public Pose2D minus(Pose2D p2) {
        return new Pose2D(x - p2.x, y - p2.y, heading - p2.heading);
    }

    public Pose2D times(double d) {
        return new Pose2D(x * d, y * d, heading * d);
    }

    public void clampAbs(Pose2D p2) {
        x = Math.copySign(minAbs(x, p2.x), x);
        y = Math.copySign(minAbs(y, p2.y), y);
        heading = Math.copySign(minAbs(heading, p2.heading), heading);
    }

    public void applyFriction(Pose2D friction) {
        x = reduceUpToZero(x, friction.x);
        y = reduceUpToZero(y, friction.y);
        heading = reduceUpToZero(heading, friction.heading);
    }

    private double reduceUpToZero(double d, double reduction) {
        return d - minAbs(d, Math.copySign(reduction, d));
    }

    private double minAbs(double a, double b) {
        return Math.abs(a) < Math.abs(b) ? a : b;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose2D pose2D = (Pose2D) o;
        return MathUtil.approxEquals(pose2D.heading, heading);
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "{x: %.3f, y: %.3f, θ: %.3f}", x, y, heading);
    }

    @Override
    public Pose2D clone(){
        return new Pose2D(this.x,this.y,this.heading);
    }
}