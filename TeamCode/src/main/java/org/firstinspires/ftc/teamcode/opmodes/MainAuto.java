package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import dev.frozenmilk.sinister.loading.Pinned;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Sator;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Main Autonomous mode for Sator robot using NextFTC framework.
 */

@NextAutonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAuto extends NextOpMode {

    private final Sator robot;

    public MainAuto(Sator robot) {
        super(robot);
        this.robot = robot;
    }

    @Override
    public void start() {
        // robot object is reused across runs, so re-apply hardware setup every run
        robot.drivetrain.configure();
        robot.intake.configure();
        robot.transfer.configure();
        robot.odometry.configure();
        robot.turret.configure();

        // where the robot starts: same spot as the TeleOp reset (back against the back wall)
        if (MainTeleOp.RED_ALLIANCE) {
            robot.odometry.setPose(MainTeleOp.RED_RESET_X, MainTeleOp.RED_RESET_Y,
                    Math.toRadians(MainTeleOp.RED_RESET_HEADING_DEG));
        } else {
            robot.odometry.setPose(MainTeleOp.BLUE_RESET_X, MainTeleOp.BLUE_RESET_Y,
                    Math.toRadians(MainTeleOp.BLUE_RESET_HEADING_DEG));
        }

        // turret must be pointing straight forward when auto starts
        robot.turret.resetRevolutions();
        robot.turret.setTargetTagId(MainTeleOp.RED_ALLIANCE ? Turret.RED_TAG_ID : Turret.BLUE_TAG_ID);

        // Schedule an autonomous command sequence using Groups and Commands
        Scheduler.schedule(
                Groups.sequential(
                        // Drive forward for 1.5 seconds
                        Groups.race(
                                robot.drivetrain.driveCommand(() -> 0.5, () -> 0.0, () -> 0.0),
                                Commands.waitMs(1500)
                        ),
                        // Pause briefly
                        Commands.waitMs(500),
                        // Spin right for 1.0 second
                        Groups.race(
                                robot.drivetrain.driveCommand(() -> 0.0, () -> 0.0, () -> 0.5),
                                Commands.waitMs(1000)
                        ),
                        // Stop drivetrain
                        Commands.instant(robot.drivetrain::stop)
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Auto Heading (rad)", robot.drivetrain.getHeading());
        telemetry.update();
    }
}
