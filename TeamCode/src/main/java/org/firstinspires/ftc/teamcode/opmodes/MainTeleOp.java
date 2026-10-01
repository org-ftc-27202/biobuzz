package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.Sator;

/**
 * Main TeleOp mode for Sator robot using NextFTC framework.
 */
@NextTeleop(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends NextOpMode {

    private final Sator robot;

    public MainTeleOp(Sator robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        CommandGamepad driver = new CommandGamepad(gamepad1);

        // Schedule driving command continuously using gamepad joysticks
        Scheduler.schedule(
                robot.drivetrain.driveCommand(
                        () -> -driver.leftStickY().getValue(),
                        driver.leftStickX()::getValue,
                        driver.rightStickX()::getValue
                )
        );

        // Bind 'A' button to reset IMU heading
        driver.a().onTrue(
                Commands.instant(robot.drivetrain::resetHeading)
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading (rad)", robot.drivetrain.getHeading());
        telemetry.update();
    }
}
