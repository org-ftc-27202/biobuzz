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

@NextTeleop(name = "Main TeleOp 10/17", group = "TeleOp")
public class MainTeleOp extends NextOpMode {

    // todo: set before each match (or make a red and a blue TeleOp)
    public static final boolean RED_ALLIANCE = false;

    // short-side start poses from decode endurancebot (Pedro coordinates); also the dpad-up reset spot
    public static final double BLUE_RESET_X = 55.25, BLUE_RESET_Y = 9.0, BLUE_RESET_HEADING_DEG = 90.0;
    public static final double RED_RESET_X = 88.75, RED_RESET_Y = 9.0, RED_RESET_HEADING_DEG = 90.0;

    private final Sator robot;

    public MainTeleOp(Sator robot) {
        super(robot);
        this.robot = robot;
    }

    private void resetPose() {
        if (RED_ALLIANCE) {
            robot.odometry.setPose(RED_RESET_X, RED_RESET_Y, Math.toRadians(RED_RESET_HEADING_DEG));
        } else {
            robot.odometry.setPose(BLUE_RESET_X, BLUE_RESET_Y, Math.toRadians(BLUE_RESET_HEADING_DEG));
        }
    }

    @Override
    public void start() {
        // robot object is reused across runs, so re-apply hardware setup every run
        robot.drivetrain.configure();
        robot.intake.configure();
        robot.transfer.configure();
        robot.odometry.configure();
        robot.turret.configure();

        // keep the pose from auto; only set one if nothing has since the code was loaded
        if (!robot.odometry.hasPose()) {
            resetPose();
        }

        if (RED_ALLIANCE) {
            robot.turret.setGoal(Turret.RED_GOAL_X, Turret.RED_GOAL_Y);
        } else {
            robot.turret.setGoal(Turret.BLUE_GOAL_X, Turret.BLUE_GOAL_Y);
        }

        CommandGamepad driver = new CommandGamepad(gamepad1);
        CommandGamepad operator = new CommandGamepad(gamepad2);

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

        // 'B' turns turret auto-aim on/off
        driver.b().onTrue(robot.turret.toggleAimCommand());

        // dpad up: robot is sitting on the reset spot, so tell odometry where it is
        driver.dpadUp().onTrue(Commands.instant(this::resetPose));

        // gamepad2 dpad left/right: nudge aim 3 degrees if shots miss to one side
        operator.dpadLeft().onTrue(Commands.instant(robot.turret::nudgeLeft));
        operator.dpadRight().onTrue(Commands.instant(robot.turret::nudgeRight));

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
        telemetry.addData("Odo X / Y (in)", "%.1f / %.1f", robot.odometry.getX(), robot.odometry.getY());
        telemetry.addData("Odo Heading (deg)", "%.1f", Math.toDegrees(robot.odometry.getHeading()));
        telemetry.addData("Auto-Aim", robot.turret.isAiming() ? "ON" : "off");
        telemetry.addData("Goal In Range", robot.turret.isTargetInRange() ? "yes" : "NO - turn robot");
        telemetry.addData("Distance To Goal (in)", "%.1f", robot.turret.getDistanceToGoal());
        telemetry.addData("Turret Deg / Target", "%.1f / %.1f",
                robot.turret.getTurretDegrees(), robot.turret.getTargetDegrees());
        if (robot.turret.isSafetyStopped()) {
            telemetry.addLine("TURRET SAFETY STOP - turret went past its limits, restart OpMode");
        }
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Transfer State", robot.transfer.getState());
        telemetry.addData("Turret State", robot.turret.getState());
        telemetry.update();
    }
}
