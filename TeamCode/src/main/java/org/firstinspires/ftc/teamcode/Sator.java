package org.firstinspires.ftc.teamcode;

import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.NextRobot;
import org.jetbrains.annotations.NotNull;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

/**
 * Main robot class for Biobuzz. Implements NextRobot for automatic scanner discovery.
 */
public class Sator implements NextRobot {

    public final Drivetrain drivetrain;
    private final Set<Mechanism> mechanisms;

    public Sator() {
        drivetrain = new Drivetrain();

        Set<Mechanism> mechanismSet = new HashSet<>();
        mechanismSet.add(drivetrain);
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
