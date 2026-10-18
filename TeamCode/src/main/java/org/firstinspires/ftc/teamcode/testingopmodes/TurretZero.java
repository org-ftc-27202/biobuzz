package org.firstinspires.ftc.teamcode.testingopmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.MainTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Turret zero finder and manual tester.
 * - point the turret straight forward and copy "New TURRET_ZERO_DEG" into Turret.java
 * - after START, the left stick turns the turret with NO limits; use Min / Max to pick LEFT_BOUND / RIGHT_BOUND
 * - shows where the turret wants to be and where the game goal is
 */
@TeleOp(name = "Turret Zero / Tester", group = "Robot")
public class TurretZero extends LinearOpMode {
    private static final double MAX_MANUAL_POWER = 0.3;

    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        AnalogInput externalEncoder = hardwareMap.get(AnalogInput.class, "externalEncoder");
        CRServo turretRotation = hardwareMap.get(CRServo.class, "turretRotation");
        turretRotation.setDirection(Turret.ROTATION_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Odometry.PINPOINT_NAME);
        Odometry.applyConfig(pinpoint);
        resetPose();

        double goalX = MainTeleOp.RED_ALLIANCE ? Turret.RED_GOAL_X : Turret.BLUE_GOAL_X;
        double goalY = MainTeleOp.RED_ALLIANCE ? Turret.RED_GOAL_Y : Turret.BLUE_GOAL_Y;

        // turret must start roughly forward, same as the real code
        double lastEncoderDeg = readEncoderDegrees(externalEncoder);
        int revolutions = Turret.startingRevolutions(lastEncoderDeg);

        double minDeg = Double.POSITIVE_INFINITY;
        double maxDeg = Double.NEGATIVE_INFINITY;
        boolean lastDpadUp = false;

        while (!isStopRequested()) {
            // ---- encoder and lap counting ----
            double encoderDeg = readEncoderDegrees(externalEncoder);
            double delta = encoderDeg - lastEncoderDeg;
            if (delta < -180.0) {
                revolutions++;
            } else if (delta > 180.0) {
                revolutions--;
            }
            lastEncoderDeg = encoderDeg;

            double turretDeg = Turret.encoderToTurretDegrees(encoderDeg, revolutions);
            minDeg = Math.min(minDeg, turretDeg);
            maxDeg = Math.max(maxDeg, turretDeg);

            // ---- controls (only after START) ----
            if (opModeIsActive()) {
                // positive servo power turns the turret toward negative degrees, so stick right = turret right
                turretRotation.setPower(-gamepad1.left_stick_x * MAX_MANUAL_POWER);

                if (gamepad1.a) {
                    minDeg = turretDeg;
                    maxDeg = turretDeg;
                }
                if (gamepad1.dpad_up && !lastDpadUp) {
                    resetPose();
                }
                lastDpadUp = gamepad1.dpad_up;
            }

            // ---- goal ----
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.RADIANS);

            double goalDeg = Turret.goalAngle(x, y, heading, goalX, goalY);
            double targetDeg = Range.clip(goalDeg, Turret.LEFT_BOUND, Turret.RIGHT_BOUND);
            boolean insideLimits = turretDeg >= Turret.LEFT_BOUND && turretDeg <= Turret.RIGHT_BOUND;

            // ---- telemetry ----
            telemetry.addLine(opModeIsActive()
                    ? "Left stick: turn turret (NO LIMITS)  |  A: reset min/max  |  Dpad up: reset pose"
                    : "Point turret roughly forward, then press START");

            telemetry.addLine("--- Turret (+ = right) ---");
            telemetry.addData("Turret Deg", "%.1f", turretDeg);
            telemetry.addData("Min / Max Reached", "%.1f / %.1f", minDeg, maxDeg);
            telemetry.addData("Current Limits", "%.1f / %.1f  (%s)",
                    Turret.LEFT_BOUND, Turret.RIGHT_BOUND, insideLimits ? "inside" : "OUTSIDE");

            telemetry.addLine("--- Zero ---");
            telemetry.addData("New TURRET_ZERO_DEG", "%.1f  (when pointing straight forward)",
                    encoderDeg * Turret.PULLEY_RATIO);

            telemetry.addLine("--- Goal (" + (MainTeleOp.RED_ALLIANCE ? "red" : "blue") + ") ---");
            telemetry.addData("Robot X / Y / Heading", "%.1f / %.1f / %.1f deg", x, y, Math.toDegrees(heading));
            telemetry.addData("Game Goal Direction", "%.1f  (ignores limits)", goalDeg);
            telemetry.addData("Wants To Be (target)", "%.1f  (kept within limits)", targetDeg);
            telemetry.addData("Off From Goal", "%.1f  (+ = turret right of goal)", turretDeg - goalDeg);
            telemetry.addData("Off From Target", "%.1f", turretDeg - targetDeg);

            telemetry.addLine("--- Raw ---");
            telemetry.addData("Encoder Voltage / Deg / Laps", "%.3f / %.1f / %d",
                    externalEncoder.getVoltage(), encoderDeg, revolutions);
            telemetry.update();
        }

        turretRotation.setPower(0.0);
    }

    private static double readEncoderDegrees(AnalogInput encoder) {
        return Turret.voltageToDegrees(encoder.getVoltage());
    }

    private void resetPose() {
        if (MainTeleOp.RED_ALLIANCE) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, MainTeleOp.RED_RESET_X, MainTeleOp.RED_RESET_Y,
                    AngleUnit.DEGREES, MainTeleOp.RED_RESET_HEADING_DEG));
        } else {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, MainTeleOp.BLUE_RESET_X, MainTeleOp.BLUE_RESET_Y,
                    AngleUnit.DEGREES, MainTeleOp.BLUE_RESET_HEADING_DEG));
        }
    }
}
