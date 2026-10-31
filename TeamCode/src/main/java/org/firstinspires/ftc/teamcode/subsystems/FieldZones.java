package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Field zones: where the robot is decides which hive AprilTag the turret aims at.
 *
 * Everything here is in TILES (1 tile = 24 in, field = 6 x 6 tiles). Use decimals for half / quarter tiles.
 * (0, 0) is the back-left corner on the red side (the corner with the tape line on the back wall).
 * x = tiles to the right, y = tiles up from the back wall. Same frame as odometry, just in tiles.
 *
 * There are 4 hives with two tags each. A zone can also give the hive's position (aimAt) so encoder + odometry
 * aim at the right hive before the Limelight sees it. Without an aim spot, the zone uses the default hive
 * (Turret.GOAL_X / GOAL_Y) and the Limelight does NOT correct the encoder there (it would be the wrong hive).
 */
public final class FieldZones {
    public static final double TILE_INCHES = 24.0;

    // keep the current zone until the robot is this far into another one, so it doesn't flicker on a line
    private static final double SWITCH_MARGIN_TILES = 0.1;

    private FieldZones() {}

    public static double tilesToInches(double tiles) {
        return tiles * TILE_INCHES;
    }

    public static double inchesToTiles(double inches) {
        return inches / TILE_INCHES;
    }

    public static final class Zone {
        public final String name;
        public final double x1, y1, x2, y2; // tiles, x1 < x2, y1 < y2
        public final int tagId;
        private double aimXTiles = Double.NaN, aimYTiles = Double.NaN;

        Zone(String name, double x1, double y1, double x2, double y2, int tagId) {
            this.name = name;
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
            this.tagId = tagId;
        }

        // position of the hive this zone's tag is on, in tiles
        Zone aimAt(double xTiles, double yTiles) {
            aimXTiles = xTiles;
            aimYTiles = yTiles;
            return this;
        }

        public boolean hasAimSpot() {
            return !Double.isNaN(aimXTiles);
        }

        // aim spot in inches (odometry units), or the default hive if this zone has none
        public double aimXInches() {
            return hasAimSpot() ? tilesToInches(aimXTiles) : Turret.GOAL_X;
        }

        public double aimYInches() {
            return hasAimSpot() ? tilesToInches(aimYTiles) : Turret.GOAL_Y;
        }

        boolean contains(double xTiles, double yTiles, double margin) {
            return xTiles >= x1 - margin && xTiles <= x2 + margin && yTiles >= y1 - margin && yTiles <= y2 + margin;
        }
    }

    // ---- hive centers, in tiles ----
    // red near hive (tags 35, 36): 2.5 right, on the line between tiles 2 and 3 going up
    public static final double RED_NEAR_HIVE_X = 2.5, RED_NEAR_HIVE_Y = 2.0;
    // red far hive (tags 32, 31): red near mirrored over y = 3 tiles (72 in)
    public static final double RED_FAR_HIVE_X = 2.5, RED_FAR_HIVE_Y = 6.0 - RED_NEAR_HIVE_Y;
    // blue hives: red ones mirrored over x = 3 tiles (72 in)
    public static final double BLUE_NEAR_HIVE_X = 6.0 - RED_NEAR_HIVE_X, BLUE_NEAR_HIVE_Y = RED_NEAR_HIVE_Y; // tags 40, 39
    public static final double BLUE_FAR_HIVE_X = 6.0 - RED_FAR_HIVE_X, BLUE_FAR_HIVE_Y = RED_FAR_HIVE_Y;     // tags 43, 44

    // ---- RED: split at x = 2.5 tiles and y = 3 tiles ----
    public static final Zone[] RED = {
            new Zone("red orange", 0.0, 0.0, 2.5, 3.0, 35).aimAt(RED_NEAR_HIVE_X, RED_NEAR_HIVE_Y),
            new Zone("red green", 2.5, 0.0, 6.0, 3.0, 36).aimAt(RED_NEAR_HIVE_X, RED_NEAR_HIVE_Y),
            new Zone("red blue", 0.0, 3.0, 2.5, 6.0, 32).aimAt(RED_FAR_HIVE_X, RED_FAR_HIVE_Y),
            new Zone("red purple", 2.5, 3.0, 6.0, 6.0, 31).aimAt(RED_FAR_HIVE_X, RED_FAR_HIVE_Y),
    };

    // ---- BLUE: mirror of red, split at x = 3.5 tiles and y = 3 tiles ----
    public static final Zone[] BLUE = {
            new Zone("blue orange", 3.5, 0.0, 6.0, 3.0, 40).aimAt(BLUE_NEAR_HIVE_X, BLUE_NEAR_HIVE_Y),
            new Zone("blue green", 0.0, 0.0, 3.5, 3.0, 39).aimAt(BLUE_NEAR_HIVE_X, BLUE_NEAR_HIVE_Y),
            new Zone("blue blue", 3.5, 3.0, 6.0, 6.0, 43).aimAt(BLUE_FAR_HIVE_X, BLUE_FAR_HIVE_Y),
            new Zone("blue purple", 0.0, 3.0, 3.5, 6.0, 44).aimAt(BLUE_FAR_HIVE_X, BLUE_FAR_HIVE_Y),
    };

    /**
     * Zone for a robot position (inches, from odometry). Stays in the current zone near a boundary.
     * Returns the current zone (or null) if the robot is somehow off the field.
     */
    public static Zone find(boolean red, double xInches, double yInches, Zone current) {
        double x = inchesToTiles(xInches);
        double y = inchesToTiles(yInches);
        Zone[] zones = red ? RED : BLUE;

        boolean currentIsThisAlliance = false;
        for (Zone zone : zones) {
            if (zone == current) {
                currentIsThisAlliance = true;
                break;
            }
        }
        if (current != null && currentIsThisAlliance && current.contains(x, y, SWITCH_MARGIN_TILES)) {
            return current;
        }
        for (Zone zone : zones) {
            if (zone.contains(x, y, 0.0)) {
                return zone;
            }
        }
        return currentIsThisAlliance ? current : null;
    }
}
