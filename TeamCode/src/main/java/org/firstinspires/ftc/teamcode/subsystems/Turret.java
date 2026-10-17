package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.hardware.actuators.NextCRServo;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Turret implements Mechanism {
    private final NextMotor turretWheel = new NextMotor("turret");
    private final NextServo turretRotation = new NextServo("turretRotation");
    private final NextServo ballStopper = new NextServo("ballStopper");
    private final NextCRServo transferWheel = new NextCRServo("transferWheel");

    public enum State {
        OFF(0.0, 0.675),
        ON(1.0, 0.675),
        SHOOT(1.0, 0.535);

        public final double power;
        public final double hoodPos;

        State(double power, double hoodPos) {
            this.power = power;
            this.hoodPos = hoodPos;
        }
    }

    private State currentState = State.OFF;

    public Turret() {
        turretWheel.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        turretWheel.setDirection(NextMotor.Direction.REVERSE);
        transferWheel.setDirection(NextMotor.Direction.REVERSE);
    }

    public void setPower(double power) {
        turretWheel.setThrottle(power);
        transferWheel.setPower(power);
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
            case OFF:
			case SHOOT:
				setState(State.ON);
                break;
            case ON:
                setState(State.SHOOT);
                break;
		}
    }

    public Command setStateCommand(State state) {
        return Commands.instant(() -> setState(state));
    }
    public Command toggleCommand() {
        return Commands.instant(this::toggle);
    }

    @Override
    public void periodic() {
        turretWheel.setThrottle(currentState.power);
        transferWheel.setPower(currentState.power);
    }
}
