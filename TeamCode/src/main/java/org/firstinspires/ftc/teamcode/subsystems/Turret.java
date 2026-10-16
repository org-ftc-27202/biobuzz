package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

/**
 * Intake subsystem using NextFTC hardware wrappers.
 */
public class Turret implements Mechanism {

    private final NextMotor motor = new NextMotor("turret");

    private final NextServo turretRotation = new NextServo("turretRotation");
    private final NextServo ballStopper = new NextServo("ballStopper");

    public enum State {
        ON(1.0, 0.675),
        SHOOT(1.0, 0.535);

        public final double power;
        public final double hoodPos;

        State(double power, double hoodPos) {
            this.power = power;
            this.hoodPos = hoodPos;
        }
    }

    private State currentState = State.ON;

    public Turret() {
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(NextMotor.Direction.REVERSE);
    }

    public void setPower(double power) {
        motor.setThrottle(power);
    }

    public void setCover(double positon) {
        ballStopper.setPosition(positon);
    }

    public void setState(State state) {
        this.currentState = state;
        setPower(state.power);
        setCover(state.hoodPos);
    }

    public State getState() {
        return currentState;
    }

    public void toggle() {
        switch (currentState) {
            case ON:
                setState(State.SHOOT);
                break;
            case SHOOT:
                setState(State.ON);
                break;
        }
    }

    public Command toggleCommand() {
        return Commands.instant(this::toggle);
    }

    @Override
    public void periodic() {
        motor.setThrottle(currentState.power);
    }
}
