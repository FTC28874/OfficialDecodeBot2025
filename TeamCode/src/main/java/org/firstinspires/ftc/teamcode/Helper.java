package org.firstinspires.ftc.teamcode;
public class Helper{
    public static double calcDistToGoal(double robotX, double robotY) {
        double goalX = 128;
        double goalY = 128;
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

}

