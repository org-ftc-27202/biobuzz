package org.firstinspires.ftc.teamcode.configs;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Turret rotation tuning, editable live in Panels. Turret reads these every loop.
 * Values typed into Panels only last until the app restarts: copy the good ones back here.
 */
@Configurable
public class TurretPIDs {
    // ---- encoder PID: gets the turret close using the encoder + odometry (error in turret degrees) ----
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double maxIntegralSum = 0.5;
    public static double minPower = 0.006;   // smallest power that still moves the turret
    public static double tolerance = 1.0;    // degrees; stops correcting inside this
    public static double maxPower = 1.0;

    // ---- Limelight PID: takes over while the Limelight sees our tag (error in tx) ----
    public static double visionKP = 0.014;
    public static double visionKI = 0.0;
    public static double visionKD = 0.0005;
    public static double visionMaxIntegralSum = 0.5;
    // the turret needs different power to move each way, so each direction has its own settings.
    // minPower: the push never drops below this while outside the tolerance (a floor).
    // kS: extra push always ADDED in that direction, for drag that never goes away (cable, belt).
    public static double visionMinPowerCCW = 0.009; // counterclockwise (turret turning left)
    public static double visionMinPowerCW = 0.07;   // clockwise (turret turning right)
    public static double visionKsCCW = 0.0;
    public static double visionKsCW = 0.0;
    public static double visionTolerance = 0.1;  // stops correcting inside this much tx
    public static double visionMaxPower = 0.4;

    // ---- shared ----
    public static double maxPowerStep = 0.3; // most the power can change per loop (both PIDs)

    // ---- Limelight aim ----
    public static double limelightTxAim = 0.0;        // tx when the turret is aimed perfectly at the hive
    public static boolean limelightTxReversed = false; // true if the turret moves AWAY from the tag when it locks on
}
