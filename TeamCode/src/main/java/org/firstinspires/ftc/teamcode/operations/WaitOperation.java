package org.firstinspires.ftc.teamcode.operations;

public class WaitOperation extends Operation {
    public WaitOperation(float maxRuntime) {
        super("waiting...", maxRuntime);
    }

    public int operate() {
        return -1;
    }
}