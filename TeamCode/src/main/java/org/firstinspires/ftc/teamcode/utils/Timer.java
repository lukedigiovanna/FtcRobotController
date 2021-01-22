package org.firstinspires.ftc.teamcode.utils;

public class Timer {
    private long last;
    private long heldLast;

    public Timer() {
        last = System.currentTimeMillis();
        heldLast = last;
    }

    public float elapsed() {
        long current = System.currentTimeMillis();
        float elapsed = (current - heldLast) / 1000.0f;
        return elapsed;
    }

    public void hold() {
        this.heldLast = System.currentTimeMillis();
    }

    public float mark() {
        long current = System.currentTimeMillis();
        float elapsed = (current - last) / 1000.0f;
        last = current;
        return elapsed;
    }
}