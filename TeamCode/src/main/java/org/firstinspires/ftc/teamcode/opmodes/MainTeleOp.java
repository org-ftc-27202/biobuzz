package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.groups.Groups.parallel;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;

import dev.nextftc.robot.opmode.NextOpMode;
import dev.nextftc.robot.opmode.NextTeleop;
import dev.nextftc.robot.triggers.CommandGamepad;
import org.firstinspires.ftc.teamcode.Sator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

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

        // schedule driving command continuously using gamepad joysticks
        Scheduler.schedule(
                robot.drivetrain.driveCommand(
                        () -> -driver.leftStickY().getValue(),
                        driver.leftStickX()::getValue,
                        driver.rightStickX()::getValue
                )
        );

        // bind 'A' button to reset IMU heading
        driver.a().onTrue(
                Commands.instant(robot.drivetrain::resetHeading)
        );

        // intake and transfer OFF -> FORWARDS -> BACKWARDS -> FORWARDS -> etc.
        driver.rightTrigger().isOver(0.5).onTrue(
                parallel(
                        robot.intake.toggleCommand(),
                        robot.transfer.toggleCommand()
                )
        );

        // shoot!
        driver.leftBumper().onTrue(
                parallel(
                        robot.turret.setStateCommand(Turret.State.SHOOT),
                        robot.transfer.setStateCommand(Transfer.State.FORWARDS),
                        robot.intake.setStateCommand(Intake.State.FORWARDS)
                )
        );

        // stop shooting!
        driver.leftBumper().onFalse(
                parallel(
                        robot.turret.setStateCommand(Turret.State.ON),
                        robot.transfer.setStateCommand(Transfer.State.FORWARDS),
                        robot.intake.setStateCommand(Intake.State.FORWARDS)
                )
        );

        // turn off turret, transfer, and intake
        driver.dpadDown().onTrue(
                parallel(
                        robot.turret.setStateCommand(Turret.State.OFF),
                        robot.transfer.setStateCommand(Transfer.State.OFF),
                        robot.intake.setStateCommand(Intake.State.OFF)
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Heading (rad)", robot.drivetrain.getHeading());
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Transfer State", robot.transfer.getState());
        telemetry.update();
    }
}
