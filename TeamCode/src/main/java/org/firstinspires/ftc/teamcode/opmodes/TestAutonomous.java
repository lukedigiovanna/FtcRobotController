//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.*;
//
//import org.firstinspires.ftc.teamcode.hardware.MainRobotDep;
//import org.firstinspires.ftc.teamcode.operations.MoveOperation;
//import org.firstinspires.ftc.teamcode.operations.Operation;
//import org.firstinspires.ftc.teamcode.operations.OperationRunner;
//import org.firstinspires.ftc.teamcode.operations.WaitOperation;
//
//@Autonomous(name="Test Autonomous",group="Linear Opmode")
//public class TestAutonomous extends LinearOpMode {
//    private MainRobotDep robot;
//
//    public TestAutonomous() {
//
//    }
//
//    public void runOpMode() {
//        robot = new MainRobotDep(this);
//
//        telemetry.addData("Status","Initialized");
//        telemetry.update();
//
//        Operation moveForward10Inches = new MoveOperation("moving 10 inches", robot, 10, 0.15, 10);
//        Operation wait3seconds = new WaitOperation(3);
//        Operation moveBack5seconds = new Operation("moving back 5 seconds",5){
//            public int operate() {
//                robot.driveForward(-0.5);
//                return -1;
//            }
//        };
//
//        moveForward10Inches.linkOperation(wait3seconds);
//        wait3seconds.linkOperation(moveBack5seconds);
//
//        OperationRunner runner = new OperationRunner(moveForward10Inches);
//
//        waitForStart();
//        while (opModeIsActive()) {
//            runner.operate(robot);
//            String msg = runner.getCurrentDisplay();
//            telemetry.addData("Current Objective",msg);
//            robot.getHardware().printEncoderPositions(telemetry);
//            telemetry.update();
//        }
//    }
//}