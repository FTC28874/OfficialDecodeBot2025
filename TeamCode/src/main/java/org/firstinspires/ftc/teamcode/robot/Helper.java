package org.firstinspires.ftc.teamcode.robot;

public class Helper {

    public static double calcDistToGoal(double robotX, double robotY) {
        double goalX = 128;
        double goalY = 128;
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double calcTargetRPM(double dist) {
        double rpm = -0.0150214 * Math.pow(dist, 2) + 11.90772 * dist + 1365.70845;
        rpm = Math.min(Constants.MAX_SHOOTER_RPM, Math.max(Constants.MIN_SHOOTER_RPM, rpm));
        return rpm;
    }

    public static double calcHoodPos(double dist) {
        // Equation: y = -0.000110842x^2 + 0.0205728x - 0.360103
        double hoodPos = -0.000110842 * Math.pow(dist, 2) + 0.0205728 * dist - 0.360103;
        hoodPos =  Math.min(Constants.MAX_HOOD_POSITION, Math.max(Constants.MIN_HOOD_POSITION, hoodPos));
        return hoodPos;
    }

    public static double calcTurretHead(double robotHeading) {
        // Equation: y = 0.0450189x^2 + 0.286092x - 34.64081
        double turHead = 0.0450189 * Math.pow(robotHeading, 2) + 0.286092 * robotHeading - 34.64081;
        turHead = Math.min(Constants.MAX_TURRET_HEAD, Math.max(Constants.MIN_TURRET_HEAD, turHead));
        return turHead;
    }

}
