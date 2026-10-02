package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower create(HardwareMap h) {
        MecanumConfig mecanumConfig = new MecanumConfig(config -> {
            config.frontLeftName.set("frontLeft");
            config.frontRightName.set("frontRight");
            config.backLeftName.set("backLeft");
            config.backRightName.set("backRight");
            config.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            config.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            config.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            config.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        });

        PinpointConfig pinpointConfig = new PinpointConfig(config -> {
            config.name.set("pinpoint");
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.xPodOffset.set(0.0);
            config.yPodOffset.set(0.0);
        });

        ForesightConfig foresightConfig = new ForesightConfig(config -> {
            config.headingFeedback.set(Controller.pid(1.0, 0.0, 0.0));
            config.forwardTranslational.set(Controller.pid(1.0, 0.0, 0.0));
            config.strafeTranslational.set(Controller.pid(1.0, 0.0, 0.0));
            config.brake.set(Controller.pid(1.0, 0.0, 0.0));
            config.coast.set(Controller.pid(1.0, 0.0, 0.0));
            config.linearBrakeCoefficients.set(new Matrix(new double[][]{{0.0}}));
            config.quadraticBrakeCoefficients.set(new Matrix(new double[][]{{0.0}}));
            config.headingBrakeCoefficients.set(Vector2D.zero());
            config.maxAchievableForwardVelocity.set(30.0);
            config.maxAchievableStrafeVelocity.set(30.0);
            config.naturalForwardDeceleration.set(1.0);
            config.naturalStrafeDeceleration.set(1.0);
        });

        Mecanum drivetrain = new Mecanum(h, mecanumConfig);
        PinpointLocalizer localizer = new PinpointLocalizer(h, pinpointConfig);
        Foresight foresight = new Foresight(foresightConfig);

        return new Follower(localizer, drivetrain, foresight);
    }
}
