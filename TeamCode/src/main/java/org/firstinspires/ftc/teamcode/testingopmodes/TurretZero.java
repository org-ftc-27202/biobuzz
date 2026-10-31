package org.firstinspires.ftc.teamcode.testingopmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
        Servo hood = hardwareMap.get(Servo.class, "turretHood");
        double hoodPos = Turret.HOOD_DOWN;
        turretRotation.setDirection(Turret.ROTATION_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Odometry.PINPOINT_NAME);
        Odometry.applyConfig(pinpoint);
        resetPose();

        double goalX = Turret.GOAL_X;
        double goalY = Turret.GOAL_Y;

        // limelight is optional here too
        Limelight3A limelight = null;
        int tagId = MainTeleOp.RED_ALLIANCE ? Turret.RED_TAG_ID : Turret.BLUE_TAG_ID;
        try {
            limelight = hardwareMap.get(Limelight3A.class, Turret.LIMELIGHT_NAME);
            limelight.pipelineSwitch(Turret.LIMELIGHT_PIPELINE);
            limelight.setPollRateHz(100);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        // turret must start roughly forward, same as the real code
        double lastEncoderDeg = readEncoderDegrees(externalEncoder);
        int revolutions = Turret.startingRevolutions(lastEncoderDeg);

        double minDeg = Double.POSITIVE_INFINITY;
        double maxDeg = Double.NEGATIVE_INFINITY;
        boolean lastDpadUp = false;

        // fine-step power, for finding the smallest power that moves the turret (VISION_MIN_POWER / MIN_POWER)
        double stepPower = 0.0;
        double sentPower = 0.0;
        boolean lastLeftBumper = false, lastRightBumper = false;
        double lastTurretDeg = Double.NaN;
        double turretSpeed = 0.0;
        ElapsedTime speedTimer = new ElapsedTime();

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

            // turret speed (deg/s), smoothed so it's readable
            double seconds = speedTimer.seconds();
            speedTimer.reset();
            if (!Double.isNaN(lastTurretDeg) && seconds > 0) {
                turretSpeed = 0.7 * turretSpeed + 0.3 * (turretDeg - lastTurretDeg) / seconds;
            }
            lastTurretDeg = turretDeg;

            // ---- controls (only after START) ----
            if (opModeIsActive()) {
                // bumpers step the power in tiny amounts, X sets it back to 0
                if (gamepad1.right_bumper && !lastRightBumper) {
                    stepPower += 0.002;
                }
                if (gamepad1.left_bumper && !lastLeftBumper) {
                    stepPower -= 0.002;
                }
                if (gamepad1.x) {
                    stepPower = 0.0;
                }
                lastRightBumper = gamepad1.right_bumper;
                lastLeftBumper = gamepad1.left_bumper;

                // the stick overrides the step power while it's being pushed.
                // positive servo power turns the turret toward negative degrees, so stick right = turret right
                double stickPower = -gamepad1.left_stick_x * MAX_MANUAL_POWER;
                sentPower = Math.abs(gamepad1.left_stick_x) > 0.05 ? stickPower : stepPower;
                turretRotation.setPower(sentPower);

                // right stick up = raise hood (smaller servo number)
                hoodPos += 0.003 * gamepad1.right_stick_y;
                hoodPos = Range.clip(hoodPos, Math.min(Turret.HOOD_UP, Turret.HOOD_DOWN),
                        Math.max(Turret.HOOD_UP, Turret.HOOD_DOWN));
                hood.setPosition(hoodPos);

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
                    ? "Left stick: turn turret (NO LIMITS)  |  Bumpers: power -/+0.002  |  X: power 0  |  "
                    + "Right stick: hood  |  A: reset min/max  |  Dpad up: reset pose"
                    : "Point turret roughly forward, then press START");

            telemetry.addLine("--- Min power test: bump power up until the turret JUST starts moving, both ways ---");
            telemetry.addData("Turret Power (sent)", "%.3f", sentPower);
            telemetry.addData("Turret Speed (deg/s)", "%.1f  (%s)", turretSpeed,
                    Math.abs(turretSpeed) > 0.5 ? "MOVING" : "stopped");

            telemetry.addLine("--- Turret (+ = right) ---");
            telemetry.addData("Turret Deg", "%.1f", turretDeg);
            telemetry.addData("Min / Max Reached", "%.1f / %.1f", minDeg, maxDeg);
            telemetry.addData("Current Limits", "%.1f / %.1f  (%s)",
                    Turret.LEFT_BOUND, Turret.RIGHT_BOUND, insideLimits ? "inside" : "OUTSIDE");

            // this number doesn't change when you change TURRET_ZERO_DEG; Turret Deg reading ~0 at forward is the check
            telemetry.addLine("--- Zero ---");
            telemetry.addData("Put in TURRET_ZERO_DEG", "%.1f  (read with turret straight forward)",
                    encoderDeg * Turret.PULLEY_RATIO);
            telemetry.addData("Zero Check", Math.abs(turretDeg) < 3.0
                    ? "OK - Turret Deg is ~0, zero is set"
                    : "if turret is straight forward, zero is NOT set yet");

            telemetry.addLine("--- Hive (start: " + (MainTeleOp.RED_ALLIANCE ? "red" : "blue") + ") ---");
            telemetry.addData("Robot X / Y / Heading", "%.1f / %.1f / %.1f deg", x, y, Math.toDegrees(heading));
            telemetry.addData("Game Goal Direction", "%.1f  (ignores limits)", goalDeg);
            telemetry.addData("Wants To Be (target)", "%.1f  (kept within limits)", targetDeg);
            telemetry.addData("Off From Goal", "%.1f  (+ = turret right of goal)", turretDeg - goalDeg);
            telemetry.addData("Off From Target", "%.1f", turretDeg - targetDeg);

            // ---- limelight ----
            telemetry.addLine("--- Limelight (aim at the hive by hand: tx here = LIMELIGHT_TX_AIM) ---");
            String tagsSeen = "none";
            boolean ourTag = false;
            double tx = 0.0;
            if (limelight == null) {
                telemetry.addData("Limelight", "NOT IN CONFIG (name: %s)", Turret.LIMELIGHT_NAME);
            } else {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    StringBuilder ids = new StringBuilder();
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        ids.append(tag.getFiducialId()).append(' ');
                        if (tag.getFiducialId() == tagId) {
                            ourTag = true;
                            tx = tag.getTargetXDegrees();
                        }
                    }
                    if (ids.length() > 0) {
                        tagsSeen = ids.toString().trim();
                    }
                }
                telemetry.addData("Connected", limelight.isConnected() ? "yes" : "NO");
                telemetry.addData("Tags Seen", tagsSeen);
                telemetry.addData("Looking For Tag", tagId);
                if (ourTag) {
                    telemetry.addData("Limelight tx", "%.2f  (+ = hive is right of the camera)", tx);
                    telemetry.addData("Turret Angle From Vision", "%.1f  (encoder says %.1f)",
                            goalDeg - (tx - Turret.LIMELIGHT_TX_AIM), turretDeg);
                }
            }

            double distance = Math.hypot(goalX - x, goalY - y);
            telemetry.addLine("--- Hood (find the hood that scores, write {distance, hood} into HOOD_TABLE) ---");
            telemetry.addData("Distance To Hive (in)", "%.1f", distance);
            telemetry.addData("Hood Now", "%.3f", hoodPos);
            telemetry.addData("HOOD_TABLE Says", "%.3f", Turret.hoodForDistance(distance));

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
