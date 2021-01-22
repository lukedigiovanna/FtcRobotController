package org.firstinspires.ftc.teamcode.utils;

public class Vector {
    public double x, y;

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void add(double dx, double dy) {
        this.x += dx;
        this.y += dy;
    }

    public void add(Vector vector) {
        this.x += vector.x;
        this.y += vector.y;
    }

    public void set(double angle, double magnitude) {
        this.x = Math.cos(angle) * magnitude;
        this.y = Math.sin(angle) * magnitude;
    }

    public double getSquaredMagnitude() {
        return x * x + y * y;
    }

    public double getMagnitude() {
        return Math.sqrt(getSquaredMagnitude());
    }

    public void normalize() {
        double magnitude = this.getMagnitude();
        this.x /= magnitude;
        this.y /= magnitude;
    }
}
