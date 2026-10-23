package org.firstinspires.ftc.teamcode;

import dev.frozenmilk.sinister.loading.Pinned;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Main robot class for Biobuzz. Implements NextRobot for automatic scanner discovery.
 */

public class Sator implements NextRobot {

    public final Odometry odometry;
    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Transfer transfer;
    public final Turret turret;
    private final Set<Mechanism> mechanisms;


    public Sator() {
        odometry = new Odometry();
        drivetrain = new Drivetrain();
        intake = new Intake();
        transfer = new Transfer();
        turret = new Turret(odometry);

        Set<Mechanism> mechanismSet = new HashSet<>();
        mechanismSet.add(drivetrain);
        mechanismSet.add(intake);
        mechanismSet.add(transfer);
        mechanismSet.add(turret);
        this.mechanisms = Collections.unmodifiableSet(mechanismSet);
    }

    // ball stopper inactive 0.535
    // ball stopper active 0.675

    // hood fully down is 0.855
    // hood max is 0.12

    @NotNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    // runs before every mechanism's periodic, so the turret always aims with a fresh pose
    @Override
    public void periodic() {
        odometry.update();
    }
}
