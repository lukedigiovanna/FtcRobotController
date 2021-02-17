package org.firstinspires.ftc.teamcode.operations;

public class LowerSlideOperation extends Operation {
    public LowerSlideOperation(Operation... futureOps) {
        super("Raising slide", 1.2f, futureOps);
    }

    public int operate(double dt) {
        robot.setSlidePower(-0.6);
        robot.raiseWobble();
        return -1;
    }
}
