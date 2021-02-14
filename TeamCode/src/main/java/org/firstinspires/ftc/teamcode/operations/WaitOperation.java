package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class WaitOperation extends Operation {
    public WaitOperation(RobotHardware robot, float maxRuntime, Operation... futureOps) {
        super("waiting...", robot, maxRuntime, futureOps);
    }

    public int operate(double dt) {
        return -1;
    }
}