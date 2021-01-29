package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous(name="Right Blue Auto", group="Linear Opmode")
public class RB_Autonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this.hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        double last = runtime.milliseconds();
        while (opModeIsActive()) {
            telemetry.addData("Robot Angle: ",robot.getAngle());
            double now = runtime.milliseconds();
            double dt = (now - last) / 1000.0;
            last = now;

            telemetry.update();
        }
    }
}
