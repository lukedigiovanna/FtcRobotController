package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.MainRobotDep;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDep;

public class MoveOperation extends Operation {
    private MainRobotDep robot;
    private double inches, power;

    public MoveOperation(String display, MainRobotDep robot, double inches, double power, float timeout) {
        super(display, timeout);
        this.robot = robot;
        this.power = power;
        this.inches = inches;
    }

    public void init() {
        super.init();
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int addtTicks = (int)(this.inches * RobotHardwareDep.TICKS_PER_INCH);
        int blTarget = robot.backLeft.getCurrentPosition() - addtTicks,
            brTarget = robot.backRight.getCurrentPosition() + addtTicks,
            flTarget = robot.frontLeft.getCurrentPosition() - addtTicks,
            frTarget = robot.frontRight.getCurrentPosition() + addtTicks;
        robot.backLeft.setTargetPosition(blTarget);
        robot.backRight.setTargetPosition(brTarget);
        robot.frontLeft.setTargetPosition(flTarget);
        robot.frontRight.setTargetPosition(frTarget);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setPower(-power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.frontRight.setPower(power);
    }

    public int operate() {
        if (!this.robot.driveIsBusy())
            return 0;
        else
            return -1;
    }
}
