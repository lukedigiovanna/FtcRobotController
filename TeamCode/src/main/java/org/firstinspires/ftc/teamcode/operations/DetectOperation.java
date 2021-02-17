package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class DetectOperation extends Operation {

    public DetectOperation(float maxRuntime, Operation... futureOps) {
        super("detecting rings", maxRuntime, futureOps);
    }

    private String curStack = "none";
    /**
     * Once a particular stack is detected, this operation is halted and returns
     *  2 for the QUAD stack
     *  1 for the SINGLE stack
     * If the runtime is exhausted, this operation returns 0 for the NONE stack
     * @param dt
     * @return
     */
    public int operate(double dt) {
        if (this.robot.getDetection().equals("Quad"))
            this.curStack = "Quad";
        if (this.robot.getDetection().equals("Single"))
            return 1;
        else if (this.curStack.equals("Quad") && this.getPercentElapsed() > 0.9)
            return 2;
        else
            return -1;
    }
}
