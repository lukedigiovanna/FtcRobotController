//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.hardware.MainRobotDep;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//
//@TeleOp(name="Main Tele (Deprecated)b", group="Linear Opmode")
//@Deprecated
//public class MainTeleOpDep extends LinearOpMode {
//
//    public MainTeleOpDep() {
//
//    }
//
//    @Override
//    public void runOpMode() {
//        RobotHardware robot = new RobotHardware(this.hardwareMap);
//
//        telemetry.addData("Status","Initialized");
//        telemetry.updat9e();
//
//        waitForStart();
//
//        long last = System.currentTimeMillis();
//        //looping condition.. while teleop mode is active.. obvious
//        while (opModeIsActive()) {
//            double elapsed = getRuntime();
//            double ly = gamepad1.left_stick_y, lx = gamepad1.left_stick_x, rx = gamepad1.right_stick_x, ry = gamepad1.right_stick_y;
//            telemetry.addData("Joysticks","Left: ("+lx+", "+ly+") Right: ("+rx+", "+ry+")");
//            telemetry.addData("Angle: ",robot.getHardware().getAngle());
//            robot.getHardware().printEncoderPositions(telemetry);
//
//            telemetry.addData("drive train","");
//            // robot.relativeStrafe(ly, lx, rx, 0.6);
//            //robot.strafe(-ly,lx,rx,0.7);
//            //robot.driveForward(ly);
//
//            //robot.lift((gamepad1.right_trigger-gamepad1.left_trigger));
//
//            telemetry.addData("intake", gamepad1.left_bumper);
//            robot.intake(gamepad1.left_bumper);
//            //telemetry.addData("shoot", gamepad1.right_bumper);
//            //robot.fire(gamepad1.right_bumper);
//
//            telemetry.addData("Elapsed (s)",elapsed);
//            telemetry.update();
//        }
//
//        telemetry.addData("Status","Finished");
//        telemetry.update();
//    }
//}
