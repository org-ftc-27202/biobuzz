package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.hardware.sensors.NextAnalogInput;
import dev.nextftc.robot.Mechanism;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Turret implements Mechanism {
    private final NextMotor turretWheel = new NextMotor("turret");
    // continuous mode; small cache tolerance so tiny aiming powers still get sent
    private final NextCRServo turretRotation = new NextCRServo("turretRotation", 0.001);
    private final NextAnalogInput externalEncoder = new NextAnalogInput("externalEncoder");
    private final NextServo ballStopper = new NextServo("ballStopper");
    private final NextCRServo transferWheel = new NextCRServo("transferWheel");

    private final Odometry odometry;

    // ---- aiming values, copied from decode endurancebot Turret ----

    // goals in Pedro field coordinates (inches, 0-144)
    public static final double BLUE_GOAL_X = 4.0, BLUE_GOAL_Y = 143.5;
    public static final double RED_GOAL_X = 141.0, RED_GOAL_Y = 142.5;

    // external encoder
    private static final double ANALOG_MAX_VOLTS = 3.3;
    private static final double MIN_VOLTS = 0.015;
    private static final double MAX_VOLTS = 3.230;
    public static final double PULLEY_RATIO = 55.0 / 109.0; // turret degrees per encoder degree
    public static final double TURRET_ZERO_DEG = 12.7; // todo: re-find with Turret Zero tuner

    // how far the turret may turn from forward (- = left, + = right)
    public static final double LEFT_BOUND = -100.0;
    public static final double RIGHT_BOUND = 160.0;

    // Sator's rotation servo spins the opposite way from endurance's; set false if the turret runs away again
    public static final boolean ROTATION_REVERSED = false;

    // if the turret ends up this far past a bound, stop it until the next run (wrong direction or bad encoder)
    private static final double SAFETY_MARGIN_DEG = 30.0;

    // yaw PID
    private static final double KP = 0.01;
    private static final double KI = 0.005;
    private static final double KD = 0.001;
    private static final double MAX_INTEGRAL_SUM = 0.5;
    private static final double MIN_POWER = 0.006;
    private static final double TOLERANCE_DEG = 1.0;
    private static final double MAX_POWER = 0.5;
    private static final double MAX_POWER_STEP = 0.3; // per loop

    private static final double NUDGE_DEG = 3.0;

    private double goalX = BLUE_GOAL_X;
    private double goalY = BLUE_GOAL_Y;
    private boolean aiming = false;
    private boolean targetInRange = true;
    private double distanceToGoal = 0.0;
    private double nudgeDeg = 0.0;
    private boolean safetyStopped = false;

    // encoder tracking; revolutions are re-picked at the start of every run (see updateEncoder)
    private boolean encoderStarted = false;
    private double lastEncoderDeg = 0.0;
    private int revolutions = 0;
    private double turretDeg = 0.0;
    private double targetDeg = 0.0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastPower = 0.0;

    public enum State {
        OFF(0.0, 0.675),
        ON(1.0, 0.675),
        SHOOT(1.0, 0.535);

        private final double power, ballStopperPos;

        State(double power, double ballStopperPos) {
            this.power = power;
            this.ballStopperPos = ballStopperPos;
        }
    }

    private State currentState = State.OFF;

    public Turret(Odometry odometry) {
        this.odometry = odometry;
    }

    // call at the start of every run (robot object is reused across runs)
    public void configure() {
        turretWheel.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        turretWheel.setDirection(NextMotor.Direction.REVERSE);
        transferWheel.setDirection(NextMotor.Direction.REVERSE);
        turretRotation.setDirection(ROTATION_REVERSED ? NextMotor.Direction.REVERSE : NextMotor.Direction.FORWARD);

        safetyStopped = false;
        encoderStarted = false; // re-read the encoder and re-pick revolutions
        integralSum = 0.0;
        lastError = 0.0;
        lastPower = 0.0;
        pidTimer.reset();
    }

    // call at the start of auto, with the turret pointing roughly forward
    public void resetRevolutions() {
        encoderStarted = false;
    }

    public void setPower(double power) {
        turretWheel.setThrottle(power);
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

    public void nudgeRight() {
        nudgeDeg += NUDGE_DEG;
    }

    public void nudgeLeft() {
        nudgeDeg -= NUDGE_DEG;
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

        turretDeg = encoderToTurretDegrees(encoderDeg, revolutions) - nudgeDeg;
    }

    // angle the turret should point, relative to the robot's front (+ = right)
    private double calculateTarget() {
        double x = odometry.getX();
        double y = odometry.getY();
        distanceToGoal = Math.hypot(goalX - x, goalY - y);

        double target = goalAngle(x, y, odometry.getHeading(), goalX, goalY);

        targetInRange = target >= LEFT_BOUND && target <= RIGHT_BOUND;
        return Range.clip(target, LEFT_BOUND, RIGHT_BOUND);
    }

    private double calculateYawPower(double error) {
        double seconds = pidTimer.seconds();
        pidTimer.reset();

        integralSum = Range.clip(integralSum + error * seconds, -MAX_INTEGRAL_SUM, MAX_INTEGRAL_SUM);
        double derivative = seconds > 0 ? (error - lastError) / seconds : 0.0;
        lastError = error;

        double power = error * KP + integralSum * KI + derivative * KD;

        if (Math.abs(error) > TOLERANCE_DEG) {
            // small powers don't move the turret, so push at least MIN_POWER
            if (power > 0 && power < MIN_POWER) {
                power = MIN_POWER;
            } else if (power < 0 && power > -MIN_POWER) {
                power = -MIN_POWER;
            }
        } else {
            power = 0.0;
        }

        // don't change power too fast between loops
        power = lastPower + Range.clip(power - lastPower, -MAX_POWER_STEP, MAX_POWER_STEP);
        lastPower = power;
        return Range.clip(power, -MAX_POWER, MAX_POWER);
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

    @Override
    public void periodic() {
        turretWheel.setThrottle(currentState.power);
        transferWheel.setPower(currentState.power);

        updateEncoder();

        if (turretDeg < LEFT_BOUND - SAFETY_MARGIN_DEG || turretDeg > RIGHT_BOUND + SAFETY_MARGIN_DEG) {
            safetyStopped = true;
        }
        if (safetyStopped) {
            lastPower = 0.0;
            turretRotation.setPower(0.0);
            return;
        }

        targetDeg = aiming ? calculateTarget() : 0.0; // not aiming: face forward
        turretRotation.setPower(calculateYawPower(turretDeg - targetDeg));
    }
}
