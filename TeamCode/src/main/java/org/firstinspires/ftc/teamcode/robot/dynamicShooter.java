package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.MathFunctions;

public class dynamicShooter {

    private static double hoodOffset = 0;
    private static double flywheelOffset = 0;

    // Uses y = 5.55234x + 1166.91729 directly
    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(
                5.55234 * goalDist + 1166.91729,
                0,
                1400
        ) + flywheelOffset;
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(
                -6.42969e-7 * Math.pow(goalDist, 3)
                        + 0.000217142 * Math.pow(goalDist, 2)
                        - 0.0259269 * goalDist
                        + 1.40496,
                0.11,
                0.904
        ) + hoodOffset;
    }

    /**
     * Calculates true field-relative distance from the goal.
     *
     * @param robotX   Robot X relative to start (inches)
     * @param robotY   Robot Y relative to start (inches)
     * @param heading  Robot heading in radians (field CCW+)
     */
    public static double distanceFromGoal(double robotX, double robotY, double heading) {

        // Goal position in FIELD coordinates (inches)
        double goalX = 130.0;
        double goalY = 42.0;

        // Rotate robot-relative position into field coordinates
        double fieldX = robotX * Math.cos(heading) - robotY * Math.sin(heading);
        double fieldY = robotX * Math.sin(heading) + robotY * Math.cos(heading);

        // Euclidean distance to goal
        return Math.hypot(goalX - fieldX, goalY - fieldY);
    }
}
