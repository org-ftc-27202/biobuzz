package org.firstinspires.ftc.teamcode.opmodes;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;


import dev.nextftc.robot.opmode.NextAutonomous;
import dev.nextftc.robot.opmode.NextOpMode;
import org.firstinspires.ftc.teamcode.Sator;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


/**
 * Main Autonomous mode for Sator robot using NextFTC framework.
 */

@NextAutonomous(name = "Main Autonomous", group = "Autonomous")
public class MainAuto extends NextOpMode {

    private final Sator robot;
    private Follower follower;
    private Paths paths = new Paths(Alliance.RED_ALLIANCE);

    public MainAuto(Sator robot) {
        super(robot);
        this.robot = robot;

    }

    @Override
    public void start() {
        // robot object is reused across runs, so re-apply hardware setup every run
        robot.odometry.configure(); // first: making the Pedro follower sets the drive motors to FLOAT
        //robot.drivetrain.configure(); // ...so the drivetrain's BRAKE has to come after it
        robot.intake.configure();
        robot.transfer.configure();
        robot.turret.configure();
        follower = Constants.create(hardwareMap);
        // where the robot starts: same spot as the TeleOp reset (back against the back wall)
            robot.odometry.setPose(MainTeleOp.RED_RESET_X, MainTeleOp.RED_RESET_Y,
                    Math.toRadians(MainTeleOp.RED_RESET_HEADING_DEG));


        // turret must be pointing straight forward when auto starts
        robot.turret.resetRevolutions();
        robot.turret.setAlliance(MainTeleOp.RED_ALLIANCE);
        Scheduler.reset();

        // Schedule an autonomous command sequence using Groups and Commands
        schedule(
                Groups.sequential(
                        // Drive forward for 1.5 seconds
                        robot.turret.setStateCommand(Turret.State.ON),
                        robot.intake.setStateCommand(Intake.State.FORWARDS),
                        robot.transfer.setStateCommand(Transfer.State.FORWARDS),
                        // Pause briefly
                        Commands.waitMs(500),
                        // Spin right for 1.0 second
                        follow(follower, paths.toGarden()),
                        Commands.waitMs(1000),
                        follow(follower, paths.backFromGarden())

                        // Stop drivetrain
                )
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("Auto Heading (rad)", follower.localizer.pose());
        telemetry.update();
        follower.update();
        Scheduler.execute();
    }
}
