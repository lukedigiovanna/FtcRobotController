package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class StrafeOperation extends Operation {
    private double inches, power;
    private int addtTicks;

    public StrafeOperation(String display, double inches, double power, float maxRuntime, Operation... futureOps) {
        super(display, maxRuntime, futureOps);
        this.power = power;
        this.inches = inches;
        this.addtTicks = (int)(this.inches * RobotHardware.TICKS_PER_INCH);
    }

    int blTarget, brTarget, flTarget, frTarget;
    public void init() {
        super.init();
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blTarget = robot.backLeftDrive.getCurrentPosition() + addtTicks;
        brTarget = robot.backRightDrive.getCurrentPosition() - addtTicks;
        flTarget = robot.frontLeftDrive.getCurrentPosition() - addtTicks;
        frTarget = robot.frontRightDrive.getCurrentPosition() + addtTicks;
        robot.backLeftDrive.setTargetPosition(blTarget);
        robot.backRightDrive.setTargetPosition(brTarget);
        robot.frontLeftDrive.setTargetPosition(flTarget);
        robot.frontRightDrive.setTargetPosition(frTarget);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setPower(power);
        robot.frontLeftDrive.setPower(-power);
        robot.backRightDrive.setPower(-power);
        robot.frontRightDrive.setPower(power);
    }

    public int operate(double dt) {
        if (!this.robot.driveIsBusy())
            return 0;
        else    {
            return -1;
        }
    }
}
