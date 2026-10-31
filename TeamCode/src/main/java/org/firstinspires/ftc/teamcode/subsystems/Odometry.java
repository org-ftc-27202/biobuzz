package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import dev.nextftc.hardware.RobotController;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Where the robot is on the field, from the Pedro Pathing follower (Pinpoint setup in pedro/Constants).
 * Pedro passes the Pinpoint's numbers straight through: x / y in inches (x from the left wall, y from the
 * back wall), heading in radians. Same frame as FieldZones (just in inches).
 */
public class Odometry {
    // created once and kept: making it recalibrates the Pinpoint IMU and pauses 500 ms,
    // and keeping it keeps the robot's position from auto into TeleOp
    private Follower follower;

    private Pose pose = Pose.zero();
    private boolean hasPose = false;

    // call at the start of every run, BEFORE drivetrain.configure(): Pedro sets the drive motors to
    // FLOAT when the follower is made, and the drivetrain sets them back to BRAKE
    public void configure() {
        if (follower == null) {
            follower = Constants.create(RobotController.hardwareMap());
        }
    }

    /**
     * The Pedro follower, for following paths in auto.
     * WARNING: follower.update() drives the drive motors (it holds them at 0 when idle), so only call it
     * when the Drivetrain subsystem isn't also driving. This class only updates the localizer.
     */
    public Follower getFollower() {
        return follower;
    }

    // call once per loop, before anything reads the pose. only updates position, never touches the motors
    public void update() {
        if (follower == null) {
            return;
        }
        follower.localizer.update();
        pose = follower.pose();
    }

    public void setPose(double xInches, double yInches, double headingRadians) {
        if (follower == null) {
            return;
        }
        follower.setPose(new Pose(xInches, yInches, headingRadians));
        pose = follower.pose();
        hasPose = true;
    }

    // false until a pose is set after the code is loaded (auto -> teleop keeps it)
    public boolean hasPose() {
        return hasPose;
    }

    public double getX() {
        return pose.x();
    }

    public double getY() {
        return pose.y();
    }

    public double getHeading() {
        return pose.heading();
    }
}
