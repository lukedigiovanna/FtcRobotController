package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * CONTROLS:
 *   LEFT JOY STICK - STRAFING
 *   RIGHT JOY STICK - ZERO TURNING
 *   Y - MOVE FORWARD (RELATIVE TO ROBOT ORIENTATION)
 *   A - MOVE BACKWARD (RELATIVE TO ROBOT ORIENTATION)
 *   X - TOGGLE THE WOBBLE
 *   B - SET THE ROBOTS ORIENTATION TO 0 DEGREES
 *   LEFT D-PAD - DECREASE FLYWHEEL POWER
 *   RIGHT D-PAD - INCREASE FLYWHEEL POWER
 *   DOWN D-PAD - LOWER SLIDE
 *   UP D-PAD - RAISE SLIDE
 *   RIGHT TRIGGER - ACTIVATE INTAKE
 *   LEFT TRIGGER - ACTIVATE OUTTAKE
 *   RIGHT BUMPER - ACTIVATE FLYWHEEL
 *   LEFT BUMPER - SHOOT RING
 */
@TeleOp(name="Main tele",group="Linear Opmode")
public class MainTeleOp extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
     
    private final double driveSpeed = 1;
    private final double slowSpeed = 0.25;
    private final double flywheelSpeed = RobotHardware.DEFAULT_FLYWHEEL_POWER;

    boolean rightDpadClicked = false;
    boolean leftDpadClicked = false;
    boolean bDown = false;

    double flywheelPowerDelta = 0;

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
                flywheelPowerDelta -= 0.01;
//                robot.setTargetFlywheelPower(robot.getTargetFlywheelPower() - 0.01);
                rightDpadClicked = true;
            }
            else if (!gamepad1.dpad_right)
                rightDpadClicked = false;

            if(gamepad1.dpad_left && !leftDpadClicked)    {
                flywheelPowerDelta += 0.01;
//                robot.setTargetFlywheelPower(robot.getTargetFlywheelPower() + 0.01);
                leftDpadClicked = true;
            }
            else if (!gamepad1.dpad_left)
                leftDpadClicked = false;

            robot.setTargetFlywheelPower(robot.getRecommendedFlywheelPower() + flywheelPowerDelta);

            double forward = 0;
            if (gamepad1.a)
                forward-=1.0;
            if (gamepad1.y)
                forward+=1.0;


            //endregion
//            robot.strafe(drive, strafe, turn, speed, dt);
            if (forward == 0)
                robot.relativeStrafe(drive, strafe, turn, speed, dt);
            else
                robot.strafe(forward, 0, 0, 0.8, dt);

            robot.intake(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.right_bumper)
                robot.setFlywheelPower(robot.getTargetFlywheelPower());
            else
                robot.setFlywheelPower(0);

            robot.loadRing(gamepad1.left_bumper, runtime.seconds());

            if (!bDown && gamepad1.b) {
                bDown = true;
                robot.toggleWobble();
            }
            else if (!gamepad1.b) bDown = false;

            if (gamepad1.x) {
                robot.setTargetAngleDegrees(0);
            }

            robot.setSlidePower(0);
            if (gamepad1.dpad_up) {
                robot.setSlidePower(1);
            } else if (gamepad1.dpad_down) {
                robot.setSlidePower(-1);
            }


            telemetry.update();
        }
        robot.closeTfod();
    }
}
