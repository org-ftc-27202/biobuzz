package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextIMU;
import dev.nextftc.robot.Mechanism;

import java.util.function.DoubleSupplier;

public class Drivetrain implements Mechanism {
    private final NextMotor frontLeft = new NextMotor("leftFront");
    private final NextMotor backLeft = new NextMotor("leftBack");
    private final NextMotor frontRight = new NextMotor("rightFront");
    private final NextMotor backRight = new NextMotor("rightBack");
    private final NextIMU imu = new NextIMU("imu");
    private final double turnSpeed = 1.0;

    // todo: convert to pedropath drive

    public Drivetrain() {
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    // reference https://github.com/org-ftc-27202/FtcRobotController/blob/jeff/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TwoDriverTeleOpJeff.java for mecanum drive
    public void drive(double forward, double strafe, double turn) {
        turn *= turnSpeed;

        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        frontLeft.setThrottle(fl / max);
        backLeft.setThrottle(bl / max);
        frontRight.setThrottle(fr / max);
        backRight.setThrottle(br / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return Commands.infinite(() -> drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble()))
                .setEnd(endCondition -> stop());
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public double getHeading() {
        return imu.getYaw();
    }

    @Override
    public void periodic() {}
}
