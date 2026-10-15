package org.firstinspires.ftc.teamcode;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;

import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Main robot class for Biobuzz. Implements NextRobot.
 */
public class Sator implements NextRobot {

    public final Drivetrain drivetrain;
    public final Intake intake;
    private final Set<Mechanism> mechanisms;

    public Sator() {
        drivetrain = new Drivetrain();
        intake = new Intake();

        Set<Mechanism> mechanismSet = new HashSet<>();
        mechanismSet.add(drivetrain);
        mechanismSet.add(intake);
        this.mechanisms = Collections.unmodifiableSet(mechanismSet);
    }

    @NotNull
    @Override
    public Set<Mechanism> getMechanisms() {
        return mechanisms;
    }

    @Override
    public void periodic() {

    }
}
