package org.firstinspires.ftc.teamcode;

import java.util.function.DoubleUnaryOperator;

public class Helper {

    public static double calcDistToGoal(double robotX, double robotY) {
        double goalX = 128;
        double goalY = 128;
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    public static double calculateRPM(double x) { return 0.000119897 * Math.pow(x, 4) - 0.0516745 * Math.pow(x, 3) + 7.98281 * Math.pow(x, 2) - 506.0992 * x + 13201.3049; }
}
