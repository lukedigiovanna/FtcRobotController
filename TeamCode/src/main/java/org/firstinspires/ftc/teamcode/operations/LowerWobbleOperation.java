package org.firstinspires.ftc.teamcode.operations;

public class LowerWobbleOperation extends Operation {
    public LowerWobbleOperation(Operation... futureOps) {
        super("Lowering wobble", 1, futureOps);
    }

    @Override
    public int operate(double dt) {
        robot.lowerWobble();
        return -1;
    }
}
