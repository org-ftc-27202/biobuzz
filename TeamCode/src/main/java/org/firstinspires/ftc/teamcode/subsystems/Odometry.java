package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import dev.nextftc.hardware.sensors.NextPinpoint;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Tracks where the robot is on the field using the goBILDA Pinpoint.
 * Uses Pedro field coordinates like decode: inches 0-144, heading in radians, 0 = +x.
 */
public class Odometry {
    // config name and pod setup copied from Sator's tuned pedro/Constants localizerConfig
    public static final String PINPOINT_NAME = "computer";
    private final NextPinpoint pinpoint = new NextPinpoint(PINPOINT_NAME);
    private static final double FORWARD_POD_Y = -3.1236324160117803; // inches (Pedro xPodOffset)
    private static final double STRAFE_POD_X = -0.35018875842957986; // inches (Pedro yPodOffset)

    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
    private boolean hasPose = false;

    // call at the start of every run (robot object is reused across runs)
    public void configure() {
        applyConfig(pinpoint.getDevice());
    }

    // shared with test OpModes that use the Pinpoint directly
    public static void applyConfig(GoBildaPinpointDriver device) {
        // same order Pedro uses: forwardPodY, strafePodX
        device.setOffsets(FORWARD_POD_Y, STRAFE_POD_X, DistanceUnit.INCH);
        device.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        device.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
    }

    // call once per loop, before anything reads the pose
    public void update() {
        pinpoint.update();
        pose = pinpoint.getDevice().getPosition();
    }

    public void setPose(double xInches, double yInches, double headingRadians) {
        pinpoint.getDevice().setPosition(
                new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.RADIANS, headingRadians)
        );
        hasPose = true;
    }

    // false until a pose is set after the code is loaded (auto -> teleop keeps it)
    public boolean hasPose() {
        return hasPose;
    }

    public double getX() {
        return pose.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return pose.getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pose.getHeading(AngleUnit.RADIANS);
    }
}
