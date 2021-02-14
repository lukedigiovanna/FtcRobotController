package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main tele",group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
     
    private final double driveSpeed = 1;
    private final double slowSpeed = 0.25;
    private final double flywheelSpeed = RobotHardware.DEFAULT_FLYWHEEL_POWER;

    boolean rightDpadClicked = false;
    boolean leftDpadClicked = false;
    boolean yDown = false;
    
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            robot.printInformation(telemetry);
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            double drive    = -gamepad1.left_stick_y;
            double turn     = gamepad1.right_stick_x;
            double strafe   = gamepad1.left_stick_x;
            double speed;
            if(gamepad1.left_stick_button)
                speed = slowSpeed;
            else
                speed = driveSpeed;

            //region 90 degree turns

            if(gamepad1.dpad_right && !rightDpadClicked)    {
                robot.changeTurnAngle(-Math.PI/2);
                rightDpadClicked = true;
            }
            else if (!gamepad1.dpad_right)
                rightDpadClicked = false;

            if(gamepad1.dpad_left && !leftDpadClicked)    {
                robot.changeTurnAngle(Math.PI/2);
                leftDpadClicked = true;
            }
            else if (!gamepad1.dpad_left)
                leftDpadClicked = false;

            //endregion
            robot.strafe(drive, strafe, turn, speed, dt);
//            robot.relativeStrafe(drive, strafe, turn, speed, dt);
            robot.intake(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.right_bumper)
                robot.setFlywheelPower(robot.getTargetFlywheelPower());
            else
                robot.setFlywheelPower(0);

            robot.loadRing(gamepad1.left_bumper, runtime.seconds());

            if (!yDown && gamepad1.y) {
                yDown = true;
                robot.toggleWobble();
            }
            else if (!gamepad1.y) yDown = false;

            robot.setSlidePower(0);
            if (gamepad1.dpad_up) {
//                robot.setSlidePower(1);
                robot.setTargetFlywheelPower(robot.getTargetFlywheelPower() + 0.01);
            } else if (gamepad1.dpad_down) {
//                robot.setSlidePower(-1);
                robot.setTargetFlywheelPower(robot.getTargetFlywheelPower() - 0.01);
            }

            robot.printRingDistance(telemetry);

            telemetry.update();
        }
        robot.closeTfod();
    }
}
