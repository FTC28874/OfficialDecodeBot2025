package org.firstinspires.ftc.teamcode.robot;

public class DynamicShooter {

    public static double goalX = 128;
    public static double goalY = 128;

    public static double calcDistToGoal(double robotX, double robotY) {

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

    public static int calcTurretHead(double robotHeading, double robotX, double robotY) {
        double turHead = 0;
        if (Shooter.getTurretPosition() <= 0){
            turHead = -721.95 + 11.8*robotX + 2.55*robotY + 9.53*robotHeading - 0.008*Math.pow(robotX, 2) - 0.092*robotX*robotY - 0.119*robotX*robotHeading + 0.027*Math.pow(robotY, 2) + 0.003*robotY*robotHeading + 0.003*Math.pow(robotHeading, 2);
            turHead = Math.min(Constants.MAX_TURRET_HEAD, Math.max(Constants.MIN_TURRET_HEAD, turHead));

        }
        else {
            turHead = -1410.77 - 21.7*Math.pow(robotX, 2) + 42.13*robotY + 12.81*robotHeading + 0.094*Math.pow(robotX, 2) + 0.058*robotX*robotY + 0.04*robotX*robotHeading - 0.215*Math.pow(robotY, 2) - 0.143*robotY*robotHeading + 0.018*Math.pow(robotHeading, 2);
            turHead = Math.min(Constants.MAX_TURRET_HEAD, Math.max(Constants.MIN_TURRET_HEAD, turHead));
        }
        return (int)turHead;
    }


    public static void setGoalPosition(double newPositionX, double newPositionY) {
        goalX = newPositionX;
        goalY = newPositionY;
    }

}
