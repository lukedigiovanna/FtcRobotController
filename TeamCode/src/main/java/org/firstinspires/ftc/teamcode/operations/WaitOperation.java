package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class WaitOperation extends Operation {
    public WaitOperation(float maxRuntime, Operation... futureOps) {
        super("waiting...", maxRuntime, futureOps);
    }

    public int operate(double dt) {
        return -1;
    }
}