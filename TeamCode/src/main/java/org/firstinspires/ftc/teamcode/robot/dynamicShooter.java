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
}
