package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;

public class Transfer implements Mechanism {

    private final ElapsedTime msTimer = new ElapsedTime();
    private double ms = 0.0;
    private boolean launchBoot = false;
    private final NextMotor transfer = new NextMotor("transfer");

    public enum State {
        OFF(0.0),
        FORWARDS(1.0),
        BACKWARDS(-1.0);

        private final double power;

        State(double power) {
            this.power = power;
        }
    }

    private State currentState = State.OFF;

    public Transfer() {

    }

    public void configure() {
        transfer.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(NextMotor.Direction.FORWARD);
    }

    public void setPower(double power) {
        transfer.setThrottle(power);

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
			case BACKWARDS:
				setState(State.FORWARDS);
                break;
            case FORWARDS:
                setState(State.BACKWARDS);
                break;
		}
    }
    public void launch(){
        setState(State.BACKWARDS);
        msTimer.reset();
        launchBoot = true;
    }

    public Command setStateCommand(State state) {
        return Commands.instant(() -> setState(state));
    }
    public Command toggleCommand() {
        return Commands.instant(this::toggle);
    }
    public Command launchCommand(){return Commands.instant(this::launch);}

    @Override
    public void periodic() {

        transfer.setThrottle(currentState.power);
        if (launchBoot){
            setState(State.BACKWARDS);
            if (msTimer.milliseconds()>400){
                launchBoot = false;
                msTimer.reset();
                setState(State.FORWARDS);
            }

        }
    }
}
