package org.firstinspires.ftc.teamcode.operations;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class WaitOperation extends Operation {
    public WaitOperation(float maxRuntime, RobotHardware robot) {
        super("waiting...", robot, maxRuntime);
    }

    public int operate() {
        return -1;
    }
}