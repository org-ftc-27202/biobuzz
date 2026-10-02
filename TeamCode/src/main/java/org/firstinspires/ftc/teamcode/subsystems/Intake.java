package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

/**
 * Intake subsystem using NextFTC hardware wrappers.
 */
public class Intake implements Mechanism {

    private final NextMotor motor = new NextMotor("intake");

    public enum State {
        OFF(0.0),
        FORWARDS(1.0),
        BACKWARDS(-1.0);

        public final double power;

        State(double power) {
            this.power = power;
        }
    }

    private State currentState = State.OFF;

    public Intake() {
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motor.setThrottle(power);
    }

    public void setState(State state) {
        this.currentState = state;
        setPower(state.power);
    }

    public State getState() {
        return currentState;
    }

    public void toggle() {
        switch (currentState) {
            case OFF:
                setState(State.FORWARDS);
                break;
            case FORWARDS:
                setState(State.BACKWARDS);
                break;
            case BACKWARDS:
                setState(State.OFF);
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
