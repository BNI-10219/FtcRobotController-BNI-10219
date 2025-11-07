package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Tester;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Angle Coordinates Program")
public class AngleCoordinatesProgram extends AutoMainProgram {

    Follower follower;

    private Timer pathTimer, opmodeTimer;

    private AngleState angleState = AngleState.READY;


    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));
    private final Pose angle45 = new Pose(36, 72, Math.toRadians(45));
    private final Pose angle90 = new Pose(72, 72, Math.toRadians(90));
    private final Pose angle135 = new Pose (36, 72, Math.toRadians(135));
    private final Pose angle1352 = new Pose(72, 72, Math.toRadians(-135));
    private final Pose angle902 = new Pose(36, 72, Math.toRadians(-90));
    private final Pose angle452 = new Pose(72, 72, Math.toRadians(-45));

    private PathChain test45, test90, test135, test1352, test902, test452;

    private enum AngleState{
        TEST45,
        TEST90,
        TEST135,
        TEST1352,
        TEST902,
        TEST452,
        READY;
    }

    private void buildPaths(){
        test45 = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, angle45))
                .setLinearHeadingInterpolation(startPose.getHeading(), angle45.getHeading())
                .build();

        test90 = follower.pathBuilder()
                .addPath(new BezierCurve(angle45, angle90))
                .setLinearHeadingInterpolation(angle45.getHeading(), angle90.getHeading())
                .build();
        test135 = follower.pathBuilder()
                .addPath(new BezierCurve(angle90, angle135))
                .setLinearHeadingInterpolation(angle90.getHeading(), angle135.getHeading())
                .build();
        test1352 = follower.pathBuilder()
                .addPath(new BezierCurve(angle135, angle1352))
                .setLinearHeadingInterpolation(angle135.getHeading(), angle1352.getHeading())
                .build();
        test902 = follower.pathBuilder()
                .addPath(new BezierCurve(angle1352, angle902))
                .setLinearHeadingInterpolation(angle1352.getHeading(), angle902.getHeading())
                .build();
        test452 = follower.pathBuilder()
                .addPath(new BezierCurve(angle902, angle452))
                .setLinearHeadingInterpolation(angle902.getHeading(), angle452.getHeading())
                .build();
    }


    private void angleStates(){
        switch(angleState) {
            case TEST45:
                follower.followPath(test45);
                angleState = AngleState.TEST90;
                break;
            case TEST90:
                if (!follower.isBusy()) {
                    follower.followPath(test90);
                    angleState = AngleState.TEST135;
                }
                break;
            case TEST135:
                if (!follower.isBusy()) {
                    follower.followPath(test135);
                    angleState = AngleState.TEST1352;
                }
                break;
            case TEST1352:
                if (!follower.isBusy()) {
                    follower.followPath(test1352);
                    angleState = AngleState.TEST902;
                }
                break;
            case TEST902:
                if (!follower.isBusy()) {
                    follower.followPath(test902);
                    angleState = AngleState.TEST452;
                }
                break;
            case TEST452:
                if (!follower.isBusy()) {
                    follower.followPath(test452);
                    angleState = AngleState.READY;
                }
                break;
            case READY:
                break;
        }

    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        angleState = AngleState.TEST45;
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        angleStates();
        // Feedback to Driver Hub for debugging
        telemetry.addData("angle state", angleState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}

