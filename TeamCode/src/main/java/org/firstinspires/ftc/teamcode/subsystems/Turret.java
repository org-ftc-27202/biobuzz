package org.firstinspires.ftc.teamcode.subsystems;


import static dev.nextftc.units.Units.DegreesPerSecond;
import static dev.nextftc.units.Units.Minutes;
import static dev.nextftc.units.Units.Radians;
import static dev.nextftc.units.Units.Rotations;
import static dev.nextftc.units.Units.RotationsPerMinute;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.control.feedback.PIDController;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.hardware.RobotController;
import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.hardware.sensors.NextAnalogInput;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;
import dev.nextftc.units.measuretypes.Angle;
import dev.nextftc.units.measuretypes.AngularVelocity;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.configs.FlywheelPIDs;
import org.firstinspires.ftc.teamcode.configs.TurretPIDs;

public class Turret implements Mechanism {
    private final NextMotor turretWheel = new NextMotor("turret", new Angle(0.2244, Radians));
    // continuous mode; small cache tolerance so tiny aiming powers still get sent
    private final NextCRServo turretRotation = new NextCRServo("turretRotation", 0.001);
    private final NextAnalogInput externalEncoder = new NextAnalogInput("externalEncoder");
    private final NextServo ballStopper = new NextServo("ballStopper");
    private final NextCRServo transferWheel = new NextCRServo("transferWheel");
    private final NextServo hood = new NextServo("turretHood", 0.001);

    private final Odometry odometry;

    // ---- aiming values, copied from decode endurancebot Turret ----

    // hive position in Pedro field coordinates (inches): x = from left wall, y = from back wall.
    // same hive for both alliances
    public static final double GOAL_X = 55.5, GOAL_Y = 57.0;

    // external encoder
    private static final double ANALOG_MAX_VOLTS = 3.3;
    private static final double MIN_VOLTS = 0.015;
    private static final double MAX_VOLTS = 3.230;
    public static final double PULLEY_RATIO = 55.0 / 109.0; // turret degrees per encoder degree
    public static final double TURRET_ZERO_DEG = 129.7; // todo: re-find with Turret Zero tuner

    // how far the turret may turn from forward (- = left, + = right)
    public static final double LEFT_BOUND = -88.6;
    public static final double RIGHT_BOUND = 205.0;

    // Sator's rotation servo spins the opposite way from endurance's; set false if the turret runs away again
    public static final boolean ROTATION_REVERSED = false;

    // if the turret ends up this far past a bound, stop it until the next run (wrong direction or bad encoder)
    private static final double SAFETY_MARGIN_DEG = 30.0;

    // turret PID gains, powers, tolerances and Limelight aim live in configs/TurretPIDs (tunable in Panels)

    private static final double NUDGE_DEG = 3.0;

    // ---- Limelight 3A, mounted on the turret ----
    // encoder + odometry get the turret close; once the Limelight sees our tag, it takes over aiming and
    // slowly corrects the encoder so aiming stays right even after the tag is lost or the belt skips.
    public static final String LIMELIGHT_NAME = "limelight";
    public static final int LIMELIGHT_PIPELINE = 0; // todo: index of the AprilTag pipeline
    public static final int RED_TAG_ID = 39;  // todo: hive tag to aim at from the red side
    public static final int BLUE_TAG_ID = 35; // todo: hive tag to aim at from the blue side
    // the tx aim point (TurretPIDs.limelightTxAim) is the tx read when the turret is aimed perfectly;
    // find it in the tester: aim the turret at the hive by hand and read "Limelight tx"
    private static final long LIMELIGHT_MAX_STALENESS_MS = 100; // ignore old results
    // encoder correction only learns while the turret is moving slowly (fast spins blur the image)
    private static final double CORRECTION_MAX_TURRET_SPEED = 30.0; // deg/s
    private static final double CORRECTION_GAIN = 0.1;             // 0-1 per loop; smaller = smoother
    private static final double MAX_ENCODER_CORRECTION_DEG = 45.0; // never shift the encoder more than this

    // ---- hood ----
    public static final double HOOD_DOWN = 0.855; // fully down
    public static final double HOOD_UP = 0.12;    // fully up
    private static final double HOOD_NUDGE = 0.02;

    // distance to hive (inches) -> hood position, used while auto-aim is on.
    // STARTING GUESSES: decode's tuned curve (near = down, far = up) scaled onto Sator's hood range.
    // replace with real numbers: park at a distance, find the hood that scores in the tester, write it here.
    private static final double[][] HOOD_TABLE = {
            {40.0, 0.750},
            {60.0, 0.700},
            {70.0, 0.488},
            {80.0, 0.350},
            {90.0, 0.239},
            {100.0, 0.120},
    };

    private double goalX = GOAL_X;
    private double goalY = GOAL_Y;
    private boolean aiming = false;
    private boolean targetInRange = true;
    private double distanceToGoal = 0.0;
    private double nudgeDeg = 0.0;
    private double hoodNudge = 0.0;
    private double hoodPos = HOOD_DOWN;
    private boolean safetyStopped = false;

    // encoder tracking; revolutions are re-picked at the start of every run (see updateEncoder)
    private boolean encoderStarted = false;
    private double lastEncoderDeg = 0.0;
    private int revolutions = 0;
    private double turretDeg = 0.0;
    private double targetDeg = 0.0;
    private double turretSpeed = 0.0; // deg/s
    private final ElapsedTime speedTimer = new ElapsedTime();

    // limelight
    private Limelight3A limelight;
    private int tagId = BLUE_TAG_ID;
    private boolean visionLocked = false;
    private double visionTx = 0.0;
    private double encoderCorrectionDeg = 0.0; // learned from the Limelight, added to the encoder angle

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastPower = 0.0;
    private boolean usingVisionPid = false; // which controller ran last loop
    private double yawPower = 0.0;          // power sent to the rotation servo this loop (for graphs)
    private double yawError = 0.0;          // error the active PID saw this loop (for graphs)
    private double turretTarget;



    public enum State {
        OFF(0.0, 0.675),
        ON(FlywheelPIDs.turretTarget, 0.675),
        SHOOT(FlywheelPIDs.turretTarget, 0.535);

        private final double power, ballStopperPos;

        State(double turretTarget, double ballStopperPos) {
            this.power = turretTarget;
            this.ballStopperPos = ballStopperPos;
        }
    }

    private State currentState = State.OFF;

    public Turret(Odometry odometry) {
        this.odometry = odometry;
    }

    // call at the start of every run (robot object is reused across runs)
    public void configure() {
        turretWheel.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);
        turretWheel.setDirection(NextMotor.Direction.REVERSE);
        transferWheel.setDirection(NextMotor.Direction.REVERSE);
        turretRotation.setDirection(ROTATION_REVERSED ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);

        // limelight is optional: if it's missing from the config, aiming falls back to encoder + odometry
        try {
            limelight = RobotController.hardwareMap().get(Limelight3A.class, LIMELIGHT_NAME);
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.setPollRateHz(100);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        safetyStopped = false;
        encoderStarted = false; // re-read the encoder and re-pick revolutions
        integralSum = 0.0;
        lastError = 0.0;
        lastPower = 0.0;
        usingVisionPid = false;
        pidTimer.reset();
        turretWheel.getVelocityConstants()
                .withP(FlywheelPIDs.kP)
                .withI(FlywheelPIDs.kI)
                .withD(FlywheelPIDs.kD)
                .withA(FlywheelPIDs.kA)
                .withS(FlywheelPIDs.kS)
                .withV(FlywheelPIDs.kV);// Like kP, other velocity PID constants include:
// kP, kI, kD, kS, kV, kA
    }

    // call at the start of auto, with the turret pointing roughly forward
    public void resetRevolutions() {
        encoderStarted = false;
    }

    public void setPower(double power) {
        //turretWheel.setThrottle(power);
        turretTarget = power;
        transferWheel.setPower(power);
    }

    public void setBallStopperServo(double positon) {
        ballStopper.setPosition(positon);
    }

    public void setState(State state) {
        this.currentState = state;
        setPower(state.power);
        setBallStopperServo(state.ballStopperPos);
    }

    public State getState() {
        return currentState;
    }

    public void toggle() {
        switch (currentState) {
            case OFF:
            case SHOOT:
                setState(State.ON);
                break;
            case ON:
                setState(State.SHOOT);
                break;
        }
    }

    public void setGoal(double x, double y) {
        goalX = x;
        goalY = y;
    }

    public void setAiming(boolean aiming) {
        this.aiming = aiming;
    }

    public boolean isAiming() {
        return aiming;
    }

    // false when the goal is past the turret's bounds; driver needs to turn the robot
    public boolean isTargetInRange() {
        return targetInRange;
    }

    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    public double getTurretDegrees() {
        return turretDeg;
    }

    public double getTargetDegrees() {
        return targetDeg;
    }

    public boolean isSafetyStopped() {
        return safetyStopped;
    }

    // which hive tag to aim at (set from the alliance at the start of TeleOp / auto)
    public void setTargetTagId(int tagId) {
        this.tagId = tagId;
    }

    public int getTargetTagId() {
        return tagId;
    }

    public boolean isLimelightConnected() {
        return limelight != null && limelight.isConnected();
    }

    // true while the Limelight sees our tag and is steering the turret
    // true while the Limelight PID (not the encoder PID) is driving the turret
    public double getYawPower() {
        return yawPower;
    }

    // degrees for the encoder PID, tx for the Limelight PID
    public double getYawError() {
        return yawError;
    }

    public double getTurretSpeed() {
        return turretSpeed;
    }

    public boolean isUsingVisionPid() {
        return usingVisionPid;
    }

    public boolean isVisionLocked() {
        return visionLocked;
    }

    public double getVisionTx() {
        return visionTx;
    }

    public double getEncoderCorrectionDegrees() {
        return encoderCorrectionDeg;
    }

    // forget what the Limelight taught the encoder (e.g. after re-zeroing by hand)
    public void resetEncoderCorrection() {
        encoderCorrectionDeg = 0.0;
    }

    // fresh result from the Limelight that includes our tag
    private void updateVision() {
        visionLocked = false;
        if (limelight == null) {
            return;
        }
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > LIMELIGHT_MAX_STALENESS_MS) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == tagId) {
                visionTx = TurretPIDs.limelightTxReversed ? -tag.getTargetXDegrees() : tag.getTargetXDegrees();
                visionLocked = true;
                return;
            }
        }
    }

    // how far the shot is off the hive per the Limelight (+ = hive is to the right). includes the aim nudge.
    private double visionError() {
        return visionTx - TurretPIDs.limelightTxAim + nudgeDeg;
    }

    // the turret can turn past 180 deg, so the same direction can be written as a, a + 360 or a - 360.
    // pick the one inside the turret's physical range (closest to the encoder if more than one fits).
    private double physicalTurretAngle(double angle) {
        double best = Double.NaN;
        for (double candidate : new double[]{angle - 360.0, angle, angle + 360.0}) {
            boolean inRange = candidate >= LEFT_BOUND - SAFETY_MARGIN_DEG && candidate <= RIGHT_BOUND + SAFETY_MARGIN_DEG;
            if (inRange && (Double.isNaN(best) || Math.abs(candidate - turretDeg) < Math.abs(best - turretDeg))) {
                best = candidate;
            }
        }
        return Double.isNaN(best) ? AngleUnit.normalizeDegrees(angle) : best;
    }

    // the Limelight says where the hive is relative to the turret, odometry says where the hive is relative
    // to the robot. together they give the turret's true angle, which keeps the encoder honest.
    private void correctEncoderFromVision() {
        if (Math.abs(turretSpeed) > CORRECTION_MAX_TURRET_SPEED) {
            return;
        }
        double hiveFromRobot = goalAngle(odometry.getX(), odometry.getY(), odometry.getHeading(), goalX, goalY);
        double trueTurretDeg = physicalTurretAngle(hiveFromRobot - (visionTx - TurretPIDs.limelightTxAim));
        double diff = trueTurretDeg - turretDeg;

        // off by a whole encoder lap (e.g. the turret didn't start near forward): fix the lap count instead
        double turretDegPerEncoderTurn = 360.0 * PULLEY_RATIO;
        if (Math.abs(diff) > turretDegPerEncoderTurn / 2) {
            int laps = (int) Math.round(diff / turretDegPerEncoderTurn);
            revolutions += laps;
            turretDeg += laps * turretDegPerEncoderTurn;
            diff -= laps * turretDegPerEncoderTurn;
        }

        encoderCorrectionDeg = Range.clip(encoderCorrectionDeg + CORRECTION_GAIN * diff,
                -MAX_ENCODER_CORRECTION_DEG, MAX_ENCODER_CORRECTION_DEG);
    }

    public void nudgeRight() {
        nudgeDeg += NUDGE_DEG;
    }

    public void nudgeLeft() {
        nudgeDeg -= NUDGE_DEG;
    }

    public double getHoodPosition() {
        return hoodPos;
    }

    // raising the hood means a smaller servo number (HOOD_UP < HOOD_DOWN)
    public void hoodNudgeUp() {
        hoodNudge -= HOOD_NUDGE;
    }

    public void hoodNudgeDown() {
        hoodNudge += HOOD_NUDGE;
    }

    // straight-line interpolation between the HOOD_TABLE rows; holds the end values outside the table
    public static double hoodForDistance(double distance) {
        if (distance <= HOOD_TABLE[0][0]) {
            return HOOD_TABLE[0][1];
        }
        for (int i = 1; i < HOOD_TABLE.length; i++) {
            if (distance <= HOOD_TABLE[i][0]) {
                double d0 = HOOD_TABLE[i - 1][0], h0 = HOOD_TABLE[i - 1][1];
                double d1 = HOOD_TABLE[i][0], h1 = HOOD_TABLE[i][1];
                return h0 + (h1 - h0) * (distance - d0) / (d1 - d0);
            }
        }
        return HOOD_TABLE[HOOD_TABLE.length - 1][1];
    }

    private void updateHood() {
        distanceToGoal = Math.hypot(goalX - odometry.getX(), goalY - odometry.getY());

        double pos = aiming ? hoodForDistance(distanceToGoal) + hoodNudge : HOOD_DOWN;
        hoodPos = Range.clip(pos, Math.min(HOOD_UP, HOOD_DOWN), Math.max(HOOD_UP, HOOD_DOWN));
        hood.setPosition(hoodPos);
    }

    public static double voltageToDegrees(double voltage) {
        return ((voltage - MIN_VOLTS) / (MAX_VOLTS - MIN_VOLTS)) * 360.0;
    }

    // one encoder turn is less than one turret turn, so the start angle is ambiguous.
    // assume the turret starts within half an encoder turn of forward and pick the turn closest to forward.
    public static int startingRevolutions(double encoderDeg) {
        double turretDegPerEncoderTurn = 360.0 * PULLEY_RATIO;
        double startDeg = encoderDeg * PULLEY_RATIO - TURRET_ZERO_DEG;
        return (int) Math.round(-startDeg / turretDegPerEncoderTurn);
    }

    public static double encoderToTurretDegrees(double encoderDeg, int revolutions) {
        return (encoderDeg + revolutions * 360.0) * PULLEY_RATIO - TURRET_ZERO_DEG;
    }

    // direction of the goal relative to the robot's front, -180 to 180 (+ = right), ignoring the bounds
    public static double goalAngle(double robotX, double robotY, double headingRad, double goalX, double goalY) {
        double angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        return AngleUnit.normalizeDegrees(Math.toDegrees(headingRad) - angleToGoal);
    }

    // the encoder spins more than once per turret turn, so count each time it wraps past 360
    private void updateEncoder() {
        double encoderDeg = voltageToDegrees(externalEncoder.getValue() * ANALOG_MAX_VOLTS);

        if (!encoderStarted) {
            revolutions = startingRevolutions(encoderDeg);
            lastEncoderDeg = encoderDeg;
            encoderStarted = true;
        }

        double delta = encoderDeg - lastEncoderDeg;
        if (delta < -180.0) {
            revolutions++;
        } else if (delta > 180.0) {
            revolutions--;
        }
        lastEncoderDeg = encoderDeg;

        double previousDeg = turretDeg;
        turretDeg = encoderToTurretDegrees(encoderDeg, revolutions) + encoderCorrectionDeg;

        double seconds = speedTimer.seconds();
        speedTimer.reset();
        if (seconds > 0) {
            turretSpeed = (turretDeg - previousDeg) / seconds;
        }
    }

    // angle the turret should point, relative to the robot's front (+ = right).
    // the Limelight overrides encoder + odometry whenever it sees our tag.
    private double calculateTarget() {
        double target = visionLocked
                ? turretDeg + visionError()
                : physicalTurretAngle(goalAngle(odometry.getX(), odometry.getY(), odometry.getHeading(), goalX, goalY) + nudgeDeg);

        targetInRange = target >= LEFT_BOUND && target <= RIGHT_BOUND;
        return Range.clip(target, LEFT_BOUND, RIGHT_BOUND);
    }

    private double calculateYawPower(double error) {
        return calculateYawPower(error, TurretPIDs.kP, TurretPIDs.kI, TurretPIDs.kD, TurretPIDs.maxIntegralSum,
                TurretPIDs.minPower, TurretPIDs.tolerance, TurretPIDs.maxPower);
    }

    // error > 0 means drive with positive power (turret degrees go down)
    private double calculateVisionPower(double error) {
        return calculateYawPower(error, TurretPIDs.visionKP, TurretPIDs.visionKI, TurretPIDs.visionKD,
                TurretPIDs.visionMaxIntegralSum, TurretPIDs.visionMinPower, TurretPIDs.visionTolerance,
                TurretPIDs.visionMaxPower);
    }

    private double calculateYawPower(double error, double kP, double kI, double kD, double maxIntegralSum,
                                     double minPower, double tolerance, double maxPower) {
        double seconds = pidTimer.seconds();
        pidTimer.reset();

        integralSum = Range.clip(integralSum + error * seconds, -maxIntegralSum, maxIntegralSum);
        double derivative = seconds > 0 ? (error - lastError) / seconds : 0.0;
        lastError = error;

        double power = error * kP + integralSum * kI + derivative * kD;

        if (Math.abs(error) > tolerance) {
            // small powers don't move the turret, so push at least minPower
            if (power > 0 && power < minPower) {
                power = minPower;
            } else if (power < 0 && power > -minPower) {
                power = -minPower;
            }
        } else {
            power = 0.0;
        }

        // don't change power too fast between loops
        power = lastPower + Range.clip(power - lastPower, -TurretPIDs.maxPowerStep, TurretPIDs.maxPowerStep);
        lastPower = power;
        return Range.clip(power, -maxPower, maxPower);
    }

    public Command setStateCommand(State state) {
        return Commands.instant(() -> setState(state));
    }

    public Command toggleCommand() {
        return Commands.instant(this::toggle);
    }

    public Command toggleAimCommand() {
        return Commands.instant(() -> setAiming(!aiming));
    }
    public void updateTurretPID(){
        if (FlywheelPIDs.turretTarget == 0.0) {
            turretWheel.setThrottle(0.0);
        } else {
            turretWheel.setVelocitySetpoint(RotationsPerMinute.of(FlywheelPIDs.turretTarget));
        }
    }
    public double getTurretTarget(){return FlywheelPIDs.turretTarget;}
    public double getTurretVelocity(){return turretWheel.getEncoderVelocity().into(RotationsPerMinute);}
    public double getTurretPIDkV(){return turretWheel.getVelocityConstants().getKV();}


    @Override
    public void periodic() {
        transferWheel.setPower(currentState.power);

        updateHood(); // before the safety stop, so the hood keeps working even if the turret is stopped
        updateEncoder();
        updateVision();
        if (visionLocked) {
            correctEncoderFromVision();
        }
        updateTurretPID();

        if (turretDeg < LEFT_BOUND - SAFETY_MARGIN_DEG || turretDeg > RIGHT_BOUND + SAFETY_MARGIN_DEG) {
            safetyStopped = true;
        }
        if (safetyStopped) {
            lastPower = 0.0;
            yawPower = 0.0;
            turretRotation.setPower(0.0);
            return;
        }

        targetDeg = aiming ? calculateTarget() : 0.0; // not aiming: face forward

        // Limelight PID while it sees our tag (and the aim point is inside the turret's range),
        // otherwise the encoder PID. both use the same sign: positive power = turret degrees go down.
        boolean useVision = aiming && visionLocked && targetInRange;
        double error = useVision ? -visionError() : turretDeg - targetDeg;
        if (useVision != usingVisionPid) {
            // switching controllers: start fresh so there's no integral carry-over or derivative kick
            integralSum = 0.0;
            lastError = error;
            usingVisionPid = useVision;
        }
        yawError = error;
        yawPower = useVision ? calculateVisionPower(error) : calculateYawPower(error);
        turretRotation.setPower(yawPower);

        turretWheel.getVelocityConstants()
                .withP(FlywheelPIDs.kP)
                .withI(FlywheelPIDs.kI)
                .withD(FlywheelPIDs.kD)
                .withA(FlywheelPIDs.kA)
                .withS(FlywheelPIDs.kS)
                .withV(FlywheelPIDs.kV);
    }
}
