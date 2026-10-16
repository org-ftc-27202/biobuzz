package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.hardware.sensors.NextIMU;
import dev.nextftc.robot.Mechanism;

import java.util.function.DoubleSupplier;

/**
 * Mecanum Drivetrain mechanism using NextFTC hardware wrappers.
 */
public class Drivetrain implements Mechanism {

    private final NextMotor frontLeft = new NextMotor("leftFront");
    private final NextMotor backLeft = new NextMotor("leftBack");
    private final NextMotor frontRight = new NextMotor("rightFront");
    private final NextMotor backRight = new NextMotor("rightBack");

    private final NextIMU imu = new NextIMU("imu");

    public Drivetrain() {
        frontRight.setDirection(NextMotor.Direction.REVERSE);
        backRight.setDirection(NextMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot using mecanum kinematics.
     *
     * @param forward  Forward movement [-1.0, 1.0]
     * @param strafe   Side-to-side movement [-1.0, 1.0]
     * @param turn     Rotational movement [-1.0, 1.0]
     */
    public void drive(double forward, double strafe, double turn) {
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

    /**
     * Stops all drive motors.
     */
    public void stop() {
        drive(0, 0, 0);
    }

    /**
     * Returns a command to drive using the given joystick input suppliers.
     */
    public Command driveCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return Commands.infinite(() -> drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble()))
                .setEnd(endCondition -> stop());
    }

    /**
     * Resets the IMU yaw heading.
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /**
     * Gets the current IMU heading in radians.
     */
    public double getHeading() {
        return imu.getYaw();
    }

    @Override
    public void periodic() {

    }
}
