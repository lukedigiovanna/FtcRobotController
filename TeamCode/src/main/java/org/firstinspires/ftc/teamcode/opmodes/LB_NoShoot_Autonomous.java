package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.operations.MoveOperation;
import org.firstinspires.ftc.teamcode.operations.Operation;
import org.firstinspires.ftc.teamcode.operations.OperationRunner;
import org.firstinspires.ftc.teamcode.operations.StrafeOperation;
import org.firstinspires.ftc.teamcode.operations.TurnOperation;

@Autonomous(name="Left Blue NO SHOOT Auto", group="Linear Opmode")
public class LB_NoShoot_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MoveOperation moveToShoot = new MoveOperation("moving to shoot", robot, 76, 0.5, 15);
        Operation dropWobble = new Operation("drop wobble", robot, 1) {
            public int operate(double dt) {
                robot.lowerWobble();
                return -1;
            }
        };
        Operation raiseWobble = new Operation("raise slide", robot, 1) {
            public int operate(double dt) {
                robot.setSlidePower(1);
                return -1;
            }
        };
        Operation stopWobble = new Operation("stop slide", robot, 1) {
            public int operate(double dt) {
                robot.setSlidePower(0);
                return 0;
            }
        };

        OperationRunner opRun = new OperationRunner(moveToShoot);

        Operation[] operations = {
                moveToShoot, dropWobble, raiseWobble, stopWobble
        };

        for (int i = 0; i < operations.length-1; i++)
            operations[i].linkOperation(operations[i+1]);

        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ",robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            telemetry.addData("Delta Time", dt);

            opRun.operate(robot, dt);

            robot.printEncoderPositions(telemetry);
            telemetry.addData("Runner Status",opRun.getCurrentDisplay());

            telemetry.update();
        }
    }
}
