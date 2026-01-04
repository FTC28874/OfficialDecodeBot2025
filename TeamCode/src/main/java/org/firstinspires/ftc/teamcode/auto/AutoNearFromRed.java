package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Point;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AUTO: RED - Near Goal", group = "Autonomous")
public class AutoNearFromRed extends OpMode {
    private Follower follower;
    private Timer actionTimer;
    private int pathState = 0;

    // --- 1. POSES ---
    // Using Pose directly is cleaner than a separate Record for this scale
    private final Pose startPose = new Pose(74, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(74, 80, Math.toRadians(0));
    private final Pose intake1Pose = new Pose(120, 84, Math.toRadians(0));
    private final Pose intake2Pose = new Pose(120, 108, Math.toRadians(0));
    private final Pose parkPose = new Pose(100, 72, Math.toRadians(0));

    // --- 2. PATHS ---
    private PathChain startToShoot, shootToIntake1, intake1ToIntake2, toPark;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        actionTimer = new Timer();
        buildPaths();
    }

    private void buildPaths() {
        // Simple straight line
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(shootPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Example of a Curved Path (BezierCurve)
        // Requires: Start Point, Control Point (the "pull"), and End Point
        shootToIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(shootPose), 
                        new Point(100, 90), // Control Point
                        new Point(intake1Pose)
                ))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .build();

        intake1ToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake1Pose), new Point(intake2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(intake2Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), parkPose.getHeading())
                .build();
    }

    // --- 3. STATE MACHINE HELPER ---
    public void setPathState(int state) {
        pathState = state;
        actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousControl();

        // Telemetry for debugging
        telemetry.addData("State", pathState);
        telemetry.addData("Path Busy", follower.isBusy());
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }

    // --- 4. CENTRAL CONTROL ---
    private void autonomousControl() {
        switch (pathState) {
            case 0: // Move to Shoot
                follower.followPath(startToShoot);
                setPathState(1);
                break;

            case 1: // Wait for arrival + DO_SHOOTING
                if (!follower.isBusy()) {
                    // Start shooting hardware here
                    if (actionTimer.getElapsedTimeSeconds() > 1.0) { // Duration of shoot
                        setPathState(2);
                    }
                }
                break;

            case 2: // Move to Intake
                follower.followPath(shootToIntake1);
                setPathState(3);
                break;

            case 3: // Wait for arrival + DO_INTAKE
                if (!follower.isBusy()) {
                    // Start intake hardware here
                    if (actionTimer.getElapsedTimeSeconds() > 1.0) {
                        setPathState(4);
                    }
                }
                break;

            case 4: // Move to Intake 2
                follower.followPath(intake1ToIntake2);
                setPathState(5);
                break;

            case 5: // Wait for arrival
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6: // Final Parking
                follower.followPath(toPark);
                setPathState(7);
                break;

            case 7: // Finish
                if (!follower.isBusy()) {
                    telemetry.addLine("Done!");
                }
                break;
        }
    }
}