package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.actuators.NextServo;
import dev.nextftc.robot.Mechanism;

public class Intake implements Mechanism {
    private final NextMotor intake = new NextMotor("intake");
    private final NextServo scooper = new NextServo("scooperServo");
    private final double SCOOPER_STORE = .9;
    private final double SCOOPER_COLLECT =  0.315;

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

    public Intake() {

    }

    public void configure() {
        intake.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(NextMotor.Direction.REVERSE);
    }

    public void setPower(double power) {
        intake.setThrottle(power);
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
    public void scooperCollect(){
        scooper.setPosition(SCOOPER_COLLECT);
    }
    public void scooperStore(){
        scooper.setPosition(SCOOPER_STORE);
    }
    public NextServo getScooper(){
        return scooper;
    }
    public Command setStateCommand(State state) {
        return Commands.instant(() -> setState(state));
    }
    public Command toggleCommand() {
        return Commands.instant(this::toggle);
    }


    public Command setScooperCollect(){ return Commands.instant(this::scooperCollect);}
    public Command setScooperStore(){ return Commands.instant(this::scooperStore);}


    @Override
    public void periodic() {
        intake.setThrottle(currentState.power);
    }
}
