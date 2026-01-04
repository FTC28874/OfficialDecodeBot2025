package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AUTO: RED - Near Goal", group = "Linear OpMode")
public class AutoNearFromRed extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Pose startPose;
    private Pose shootPose;

    private record Spot(double x, double y) {}
    private Spot start = new Spot(74, 9);
    private Spot shoot = new Spot(74, 80);
    private Spot intake1 = new Spot(120, 84);
    private Spot intake2 = new Spot(120, 84+24);
    private Spot intake3 = new Spot(120, 84+48);
    private Spot gate = new Spot(120, 72);
    private Spot park = new Spot(100, 72);

    private void initPoses() {
        startPose = new Pose(start.x, start.y, Math.toRadians(90));
        shootPose = new Pose(shoot.x, shoot.y, Math.toRadians(0));
        intake1Pose = new Pose(intake1.x, intake1.y, Math.toRadians(0));
        intake2Pose = new Pose(intake2.x, intake2.y, Math.toRadians(0));
        intake3Pose = new Pose(intake3.x, intake3.y, Math.toRadians(0));
        gatePose = new Pose(gate.x, gate.y, Math.toRadians(0));
        parkPose = new Pose(park.x, park.y, Math.toRadians(0));
    }

    private Path path;
    private PathChain startToShoot,
            shootToIntake1,
            intake1ToIntake2,
            intake2ToIntake3,
            intake3ToGate,
            gateToPark;



    private void buildPaths() {
        initPoses();
        startToShoot = buildLinePath(startPose, shootPose); 
        shootToIntake1 = buildLinePath(shootPose, intake1Pose);
        intake1ToIntake2 = buildLinePath(intake1Pose, intake2Pose);
        intake2ToIntake3 = buildLinePath(intake2Pose, intake3Pose);
        intake3ToGate = buildLinePath(intake3Pose, gatePose);
        gateToPark = buildLinePath(gatePose, parkPose);
    }

    private PathChain buildLinePath(Pose p1, Pose p2) {
        path = follower.pathBuilder()
            .addPath(new BezierLine(p1, p2))
            .setLinearHeadingInterpolation(p1.getHeading(), p2.getHeading())
            .build();

        return path;
    }

    // Need to fix: BezierCurve syntax is not correct
    private void buildCurvePath(Pose p1, Pose p2) {
        path = follower.pathBuilder()
            .addPath(new BezierCurve(p1, p2))
            .build();   
    }

    private void statePathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startToShoot, true);
                pathState = 1;
                break;
            case 1:
                follower.followPath(shootToIntake1, true);
                pathState = 2;
                break;
            case 2:
                follower.followPath(intake1ToIntake2, true);
                pathState = 3;
                break;
            case 3:
                follower.followPath(intake2ToIntake3, true);
                pathState = 4;
                break;
            case 4:
                follower.followPath(intake3ToGate, true);
                pathState = 5;
                break;
            case 5: 
                follower.followPath(gateToPark, true);
                pathState = 6;
                break;
            default:
                break;
        }
    }


            
    @Override
    public void runOpMode() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        buildPaths();
        pathState = 0;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            statePathUpdate();
            telemetry.addData("State", pathState);
            telemetry.addData("X Position", follower.getPose().getX());
            telemetry.addData("Y Position", follower.getPose().getY());
            telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }

}