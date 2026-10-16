package org.firstinspires.ftc.teamcode.testingopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Ball Stopper Tuning", group = "Robot")

public class BallStopperTuner extends LinearOpMode {
    Servo ballStopper;

    @Override
    public void runOpMode() {
        ballStopper = hardwareMap.get(Servo.class, "ballStopper");

        telemetry.addData("Ball Stopper", "Press START to continue...");
        telemetry.update();

        waitForStart();

        double servoPos = 0.2;

        while (opModeIsActive()) {
            servoPos -= 0.0002 * gamepad1.left_stick_y;

            ballStopper.setPosition(servoPos);

            telemetry.addData("Ball Stopper", "Pos: " + servoPos);
            telemetry.update();
        }
    }
}