package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.hardware.MainRobotDep;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.RobotHardwareDep;

public class MoveOperation extends Operation {
    private RobotHardware robot;
    private double inches, power;
    private int addtTicks;

    public MoveOperation(String display, RobotHardware robot, double inches, double power, float timeout) {
        super(display, timeout);
        this.robot = robot;
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
        brTarget = robot.backRightDrive.getCurrentPosition() + addtTicks;
        flTarget = robot.frontLeftDrive.getCurrentPosition() + addtTicks;
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
        robot.frontLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
    }

    public int operate() {
        if (!this.robot.driveIsBusy())
            return 0;
        else    {
//            robot.backLeftDrive.setPower(+0.1);
//            robot.frontLeftDrive.setPower(+0.1);
//            robot.backRightDrive.setPower(+0.1);
//            robot.frontRightDrive.setPower(+0.1);
//            robot.setProtectedPower(((blTarget - robot.backLeftDrive.getCurrentPosition())/(double)addtTicks)*power,
//                    ((flTarget - robot.frontLeftDrive.getCurrentPosition())/(double)addtTicks)*power,
//                    ((brTarget - robot.backRightDrive.getCurrentPosition())/(double)addtTicks)*power,
//                    ((frTarget - robot.frontRightDrive.getCurrentPosition())/(double)addtTicks)*power);
            return -1;
        }
    }
}
