package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;

import dev.frozenmilk.sinister.loading.Pinned;
import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Sator;

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
        robot.odometry.configure();
        robot.turret.configure();

        // where the robot starts, in Pedro coordinates (decode endurancebot values)
        // blue short: 30, 136, 0 deg     red short: 114, 136, 180 deg
        // blue far: 55.25, 9, 90 deg     red far:   88.75, 9, 90 deg
        robot.odometry.setPose(30.0, 136.0, Math.toRadians(0));

        // turret must be pointing straight forward when auto starts
        robot.turret.resetRevolutions();

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
