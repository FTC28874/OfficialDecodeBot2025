package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DynamicShooter {

    public static double goalX = 128;
    public static double goalY = 128;

    private static DcMotorEx turret = null;

    private final double motorEncoderTicksPerRev = 28 * 19.2;
    public static void init(HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotorEx.class, "turret");





    }

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
        turHead = -307.1978 + (-1.6701)*robotX + (3.0168)*robotY + (5.5425)*robotHeading;
        turHead = Math.min(Constants.MAX_TURRET_HEAD, Math.max(Constants.MIN_TURRET_HEAD, turHead));
        return (int)turHead;
    }


    public static void setGoalPosition(double newPositionX, double newPositionY) {
        goalX = newPositionX;
        goalY = newPositionY;
    }

    public static void resetTurretEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
