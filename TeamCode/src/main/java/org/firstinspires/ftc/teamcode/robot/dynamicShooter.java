package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.Shooter;

public class dynamicShooter {
    private static double hoodOffset = 0;
    private static double flywheelOffset = 0;


            public static double flywheelSpeed(double goalDist){
                return MathFunctions.clamp(0.0189681 * Math.pow(goalDist, 2) + 0.654275 * goalDist + 725.56772, 0, 1400) + flywheelOffset; //in ticks per second and prob not correct

            }
            public static double hoodAngle( double goalDist) {
                return MathFunctions.clamp(-6.42969e-7 * Math.pow(goalDist, 3) + 0.000217142 * Math.pow(goalDist, 2) - 0.0259269 * goalDist + 1.40496, 0.11, 0.904) + hoodOffset;
            }




}

