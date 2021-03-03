package org.firstinspires.ftc.teamcode.math;

import java.util.Locale;

public class Vector2D{

    public double x;
    public double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D() {
        this(0, 0);
    }

    public Vector2D rotated(double angle) {
        double sina = Math.sin(angle);
        double cosa = MathUtil.cosFromSin(sina, angle);
        return new Vector2D(x * cosa - y * sina, x * sina + y * cosa);
    }

    public Vector2D rotatedCW(double angle) {
        double sina = Math.sin(angle);
        double cosa = MathUtil.cosFromSin(sina, angle);
        return new Vector2D(x * cosa + y * sina, -x * sina + y * cosa);
    }

    public Vector2D normalize() {
        double r = radius();
        if (r != 0) {
            return new Vector2D(x / r, y / r);
        }
        return this;
    }

    public Vector2D plus(Vector2D p) {
        return new Vector2D(this.x + p.x, this.y + p.y);
    }

    public Vector2D minus(Vector2D p) {
        return new Vector2D(this.x - p.x, this.y - p.y);
    }

    public double atan() {
        return Math.atan2(y, x);
    }

    public double acot() {
        return Math.atan2(x, y);
    }

    public double radius() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D times(double d) {
        return new Vector2D(x * d, y * d);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Vector2D vector2D = (Vector2D) o;
        return MathUtil.approxEquals(vector2D.x, x) &&
                MathUtil.approxEquals(vector2D.y, y);
    }

    @Override
    public int hashCode() {
        return Double.valueOf(x).hashCode() ^ Double.valueOf(y).hashCode();
    }

    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "(%.1f, %.1f)", x, y);
    }

    @Override
    public Vector2D clone(){
        return new Vector2D(this.x,this.y);
    }

}