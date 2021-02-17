package org.firstinspires.ftc.teamcode.operations;

public class ShootRingOperation extends Operation {
    public ShootRingOperation(Operation... futureOps) {
        super("Shooting ring", 0.8f, futureOps);
    }

    public int operate(double dt) {
        this.robot.forceLoadRing();
        return -1;
    }
}
