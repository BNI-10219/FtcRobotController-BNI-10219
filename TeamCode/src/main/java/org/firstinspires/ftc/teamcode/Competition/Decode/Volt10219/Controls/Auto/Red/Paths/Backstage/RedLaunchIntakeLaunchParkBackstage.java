package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Red Launch Intake Launch Park Backstage")
public class RedLaunchIntakeLaunchParkBackstage extends RedAlliance {

    Follower follower;

    private PathState pathState = PathState.READY;
    private LaunchState launchState = LaunchState.READY;

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer;

    private final Pose startPose = new Pose(120, 132, Math.toRadians(215));
    private final Pose launch = new Pose(84, 84, Math.toRadians(225));
    private final Pose intake = new Pose(108, 36, Math.toRadians(0));
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(90));
    private final Pose launchTwoPull = new Pose(72, 48, 157);
    private final Pose park = new Pose(108, 36, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    private void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        intakePath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        intakePickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickup))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickup.getHeading())
                .build();
        launchTwoPath = follower.pathBuilder()
                .addPath(new BezierCurve(intakePickup, launchTwoPull, launch))
                .setLinearHeadingInterpolation(intakePickup.getHeading(), launch.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();

    }

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        intakeTimer = new Timer();
        intakeTimer.resetTimer();
        waitTimer = new Timer();
        waitTimer.resetTimer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pathTimer = new Timer();
        pathTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.getHeading();
    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVETOLAUNCH;
        launchZone = LaunchZone.NONE;
    }

    public enum LaunchState {OUTTAKE, INTAKEONE, WAITONE, INTAKETWO, WAITTWO, INTAKETHREE, WAIT, READY}
    public enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE, PICKUP, LAUNCHPOSTWO, LAUNCHTWO, PARK, READY, WAIT;}
    @Override
    public void loop() {

        switch(pathState){
            case DRIVETOLAUNCH:
                follower.followPath(launchOne);
                pathState = PathState.LAUNCH;
                intakeTimer.resetTimer();
                break;
            case LAUNCH:
                launchState = LaunchState.OUTTAKE;
                switch(launchState) {
                    case OUTTAKE:
                        Bot.ballLaunchV();
                        if(!follower.isBusy()){
                            Bot.ballOuttake();
                            Bot.artifactPushDown();
                            launchState = LaunchState.WAIT;
                            break;
                        }

                        break;
                    case WAIT:
                        if (waitTimer.getElapsedTimeSeconds() > 1) {
                            intakeTimer.resetTimer();
                            Bot.intakeStop();
                            launchState = LaunchState.INTAKEONE;
                            break;
                        }
                        break;
                    case INTAKEONE:
                        Bot.ballIntake();
                        if (intakeTimer.getElapsedTimeSeconds() > 1) {
                            Bot.intakeStop();
                            waitTimer.resetTimer();
                            launchState = LaunchState.WAITONE;
                            break;
                        }
                        break;
                    case WAITONE:
                        if (waitTimer.getElapsedTimeSeconds() > 1) {
                            intakeTimer.resetTimer();
                            launchState = LaunchState.INTAKETWO;
                            break;
                        }
                        break;
                    case INTAKETWO:
                        Bot.ballIntake();
                        if (intakeTimer.getElapsedTimeSeconds() > 1) {
                            Bot.intakeStop();
                            waitTimer.resetTimer();
                            launchState = LaunchState.WAITTWO;
                            break;
                        }
                        break;
                    case WAITTWO:
                        if (waitTimer.getElapsedTimeSeconds() > 1) {
                            intakeTimer.resetTimer();
                            launchState = LaunchState.INTAKETHREE;
                            break;
                        }
                        break;
                    case INTAKETHREE:
                        Bot.ballIntake();
                        Bot.artifactPushUp();
                        if (intakeTimer.getElapsedTimeSeconds() > 1) {
                            Bot.intakeStop();
                            waitTimer.resetTimer();
                            launchState = LaunchState.WAITTWO;
                            break;
                        }
                        break;
                }
                waitTimer.resetTimer();
                pathState = PathState.READY;
                break;
            case WAIT:
                if(waitTimer.getElapsedTimeSeconds()> 5000){
                    pathState = PathState.INTAKE;
                    break;
                }
                break;
            case INTAKE:
                if(!follower.isBusy()) {
                    follower.followPath(intakePath);
                    pathState = PathState.READY;
                    break;
                }
                break;
            case PICKUP:
                if(!follower.isBusy()){
                    follower.followPath(intakePickupPath);
                    pathState = PathState.LAUNCHPOSTWO;
                    break;
                }
                break;
            case LAUNCHPOSTWO:
                if(!follower.isBusy()){
                    follower.followPath(launchTwoPath);
                    pathState = PathState.LAUNCHTWO;
                    break;
                }
                break;
            case LAUNCHTWO:
                if(follower.isBusy()){
                    launchZone = LaunchZone.V;
                    Bot.ballOuttake();
                    intakeTimer.resetTimer();
                    if(intakeTimer.getElapsedTimeSeconds()> .1){
                        Bot.intakeStop();
                    }
                    startFlyWheel();
                }
                if(!follower.isBusy()){
                    Bot.ballIntake();
                    intakeTimer.resetTimer();
                    if(intakeTimer.getElapsedTimeSeconds()>.1){
                        Bot.intakeStop();
                    }
                    Bot.ballIntake();
                    intakeTimer.resetTimer();
                    if(intakeTimer.getElapsedTimeSeconds() > .1){
                        Bot.artifactPushDown();
                    }
                    pathState = PathState.PARK;
                }
                break;
            case PARK:
                if(!follower.isBusy()) {
                    follower.followPath(parkPath);
                    pathState = PathState.READY;
                }
                break;
            case READY:
                break;
        }


        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("launch state", launchState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Is Busy: ", follower.isBusy());
        telemetry.update();
    }

    public void stop() {}
}


