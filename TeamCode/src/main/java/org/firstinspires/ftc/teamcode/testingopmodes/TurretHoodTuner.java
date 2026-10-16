package org.firstinspires.ftc.teamcode.testingopmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Turret Hood Tuning", group = "Robot")

public class TurretHoodTuner extends LinearOpMode {
    Servo turretHood;

    @Override
    public void runOpMode() {
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        telemetry.addData("Turret Hood", "Press START to continue...");
        telemetry.update();

        waitForStart();

        double servoPos = 0.2;

        while (opModeIsActive()) {
            servoPos -= 0.0002 * gamepad1.left_stick_y;

            turretHood.setPosition(servoPos);

            telemetry.addData("Turret Hood", "Pos: " + servoPos);
            telemetry.update();
        }
    }
}