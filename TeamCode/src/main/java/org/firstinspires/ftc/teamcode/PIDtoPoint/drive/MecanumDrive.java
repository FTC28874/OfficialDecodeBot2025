// Combined and extended Mecanum drive class with:
// - GoBILDA Pinpoint Odometry integration
// - Manual/Auto modes
// - Linear interpolated point-to-point movement with heading

package org.firstinspires.ftc.teamcode.PIDtoPoint.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    private DcMotor driveFL, driveFR, driveBL, driveBR;
    private GoBildaPinpointDriver odo;

    public enum DriveMode { MANUAL, AUTO }

    private DriveMode mode = DriveMode.MANUAL;

    private Pose2D targetPose = null;
    private double kP_position = 0.005; //0.01
    private double kP_heading = 0.02; //0.02
    private double maxPower = 0.6;

    public MecanumDrive(HardwareMap hardwareMap) {
        driveFL = hardwareMap.get(DcMotor.class, "driveFL");
        driveFR = hardwareMap.get(DcMotor.class, "driveFR");
        driveBL = hardwareMap.get(DcMotor.class, "driveBL");
        driveBR = hardwareMap.get(DcMotor.class, "driveBR");

        driveFL.setDirection(DcMotor.Direction.REVERSE);
        driveFR.setDirection(DcMotor.Direction.FORWARD);
        driveBL.setDirection(DcMotor.Direction.REVERSE);
        driveBR.setDirection(DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-48, -156, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        odo.resetPosAndIMU();
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveFL.setZeroPowerBehavior(behavior);
        driveFR.setZeroPowerBehavior(behavior);
        driveBL.setZeroPowerBehavior(behavior);
        driveBR.setZeroPowerBehavior(behavior);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        driveFL.setPower(fl);
        driveFR.setPower(fr);
        driveBL.setPower(bl);
        driveBR.setPower(br);
    }

    public void drive(double axial, double lateral, double yaw) {
            double powerFL = axial + lateral + yaw;
            double powerFR = axial - lateral - yaw;
            double powerBL = axial - lateral + yaw;
            double powerBR = axial + lateral - yaw;

            double max = Math.max(Math.abs(powerFL), Math.abs(powerFR));
            max = Math.max(max, Math.abs(powerBL));
            max = Math.max(max, Math.abs(powerBR));

            if (max > 1.0) {
                powerFL /= max;
                powerFR /= max;
                powerBL /= max;
                powerBR /= max;
            }

            setMotorPowers(powerFL, powerFR, powerBL, powerBR);
    }

    public void driveFieldCentric(double axial, double lateral, double yaw) {
        odo.update();  // Update heading from odometry

        double heading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Rotate joystick input vector by -heading (field-centric)
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double fieldAxial = axial * cos - lateral * sin;
        double fieldLateral = axial * sin + lateral * cos;

        // Calculate motor powers
        double powerFL = fieldAxial + fieldLateral + yaw;
        double powerFR = fieldAxial - fieldLateral - yaw;
        double powerBL = fieldAxial - fieldLateral + yaw;
        double powerBR = fieldAxial + fieldLateral - yaw;

        // Normalize powers
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(powerFL), Math.abs(powerFR)),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))
        ));

        setMotorPowers(powerFL / max, powerFR / max, powerBL / max, powerBR / max);
    }

    public void updateAuto() {
        odo.update();
        Pose2D current = odo.getPosition();

        // --- Field-relative position error ---
        double dx = targetPose.getX(DistanceUnit.MM) - current.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - current.getY(DistanceUnit.MM);
        double headingError = targetPose.getHeading(AngleUnit.RADIANS) - current.getHeading(AngleUnit.RADIANS);
        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        // --- Rotate the (dx, dy) into ROBOT-centric frame using -robotHeading ---
        double robotHeading = current.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double axial = kP_position * (dy * cos - dx * sin);     // Forward/back
        double lateral = kP_position * (dy * sin + dx * cos);   // Strafe
        double yaw = kP_heading * headingError;

        axial = clamp(axial, -maxPower, maxPower);
        lateral = clamp(lateral, -maxPower, maxPower);
        yaw = clamp(yaw, -maxPower, maxPower);

        drive(axial, lateral, yaw);  // still uses robot-centric inputs here
    }


    public void setTargetPose(Pose2D target) {
        this.targetPose = target;
        this.mode = DriveMode.AUTO;
    }

    public void setManualMode() {
        this.mode = DriveMode.MANUAL;
    }

    public DriveMode getMode() {
        return mode;
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
