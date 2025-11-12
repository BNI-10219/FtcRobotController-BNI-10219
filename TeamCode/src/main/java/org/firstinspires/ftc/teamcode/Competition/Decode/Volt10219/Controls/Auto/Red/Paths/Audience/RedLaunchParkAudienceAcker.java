package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Audience;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Tester: Red Launch Park Audience Acker")
public class RedLaunchParkAudienceAcker extends RedAlliance {

    //   (0, 144)                          (144, 144)
    //      --------------------------------
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      |                               |
    //      ---------------------------------
    //   (0,0)                              (144, 0)


    //                90 degrees
    //                     |
    //                     |
    //  180 degrees  --------------   0 degrees
    //                     |
    //                     |
    //                180 degrees

    Follower follower;

    private PathState pathState = PathState.READY;
    private LaunchState launchState = LaunchState.READY;

    // ***** Creep Drive Variables
    protected Timer creepTimer;
    protected static final double CREEP_POWER = 0.25;
    protected static final double CREEP_TIMEOUT_S = 1.0;

    public enum PathState { DRIVETOLAUNCH, LAUNCH, INTAKE, PICKUP, LAUNCHPOSTWO, LAUNCHTWO,
        PARK, CREEP_INTAKE, FINISHED, READY, WAIT; }

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(270));
    private final Pose launch = new Pose(86, 12, Math.toRadians(248));
    private final Pose intake = new Pose(108, 36, Math.toRadians(0));
    private final Pose intakePickup = new Pose(36, 128, Math.toRadians(90));
    private final Pose launchTwoPull = new Pose(72, 48, Math.toRadians(157));
    private final Pose park = new Pose(96, 44, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    boolean scoringDone = false;

    public enum LaunchState {OUTTAKE, INTAKEONE, WAITONE, INTAKETWO, WAITTWO, INTAKETHREE, WAIT, READY, IDLE, OUTTAKEONE, OUTTAKETWO, OUTTAKETHREE}

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
        outtakeTimer = new Timer();
        outtakeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        follower.getHeading();

        //**** Creep Drive Timer Inits
        creepTimer = new Timer();
        creepTimer.resetTimer();

    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVETOLAUNCH;
        launchState = LaunchState.READY;
        launchZone = LaunchZone.NONE;
    }

    @Override
    public void loop() {

        autoPathing();
        automaticLaunch();

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

    //DOES NOT NEED TO BE CHANGED
    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:
                follower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                pathTimer.resetTimer();
                break;

            case LAUNCH:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY || launchState == LaunchState.IDLE) {
                        launchState = LaunchState.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    Bot.ballIntake();
                    follower.followPath(intakePath);
                    waitTimer.resetTimer();
                    pathState = PathState.INTAKE;
                    pathTimer.resetTimer();
                }
                break;

            case INTAKE:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    creepTimer.resetTimer();
                    pathState = PathState.CREEP_INTAKE;
                    pathTimer.resetTimer();
                }
                break;

            case CREEP_INTAKE:
                driveForwardCreep(CREEP_POWER);
                if (creepTimer.getElapsedTimeSeconds() >= CREEP_TIMEOUT_S) {
                    stopCreepDrive();
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;  // continue your normal flow
                    pathTimer.resetTimer();
                }
                break;


            case PARK:
                follower.followPath(parkPath);
                pathState = PathState.FINISHED;  // continue your normal flow
                pathTimer.resetTimer();
                break;

            case FINISHED:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
                    pathState = PathState.READY;
                }
                break;
            case READY:
                break;
        }
    }


    //LAUNCHING CODE DURING AUTO - MIGHT NEED TO BE CHANGED
    public void automaticLaunch() {
        switch(launchState) {
            case READY:
                //To change the velocity, change the numbers below
                Bot.ballLaunchBackField();;//VELOCITY for launching 1st artifact
                // Command + B to change the velocity(while the white line index thing is in the method)

                outtakeTimer.resetTimer();
                intakeTimer.resetTimer();
                break;

            case OUTTAKE:
                if(intakeTimer.getElapsedTimeSeconds()> 3) {
                    Bot.ballOuttake();
                }
                if(outtakeTimer.getElapsedTimeSeconds() > .25){
                    Bot.intakeStop();
                    Bot.ballIntake();
                }

                //To change the velocity, change the numbers below
                Bot.ballLaunchAutoBack();//VELOCITY for launching 2nd artifact
                // Command + B to change the velocity(while the white line index thing is in the method)

                Bot.artifactPushAuto();
                waitTimer.resetTimer();
                intakeTimer.resetTimer();
                launchState = LaunchState.WAIT;
                break;
            case WAIT:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    //Bot.intakeStop();
                    launchState = LaunchState.INTAKEONE;
                    //Bot.ballLaunchAutoBack();
                }
                break;
            case INTAKEONE:
                Bot.ballIntake();
                Bot.ballLaunchAutoBack();
                Bot.artifactPushDown();
                if (intakeTimer.getElapsedTimeSeconds() > 2) {
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    launchState = LaunchState.IDLE;
                    scoringDone = true;

                }
                break;
            case WAITONE:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    Bot.intakeStop();
                    launchState = LaunchState.INTAKETWO;
                }
                break;
            case INTAKETWO:
                Bot.ballIntake();
                if (intakeTimer.getElapsedTimeSeconds() > 1.5) {
                    Bot.intakeStop();
                    waitTimer.resetTimer();
                    launchState = LaunchState.IDLE;
                }
                break;
            case IDLE:
                break;

        }

    }

    // *************   Helper Methods for Creep Drive  --------
    protected void driveForwardCreep(double power) {

        Bot.flMotor.setPower(power);
        Bot.frMotor.setPower(power);
        Bot.rlMotor.setPower(power);
        Bot.rrMotor.setPower(power);
    }

    protected void driveBackwardCreep(double power) {
        Bot.flMotor.setPower(-power);
        Bot.frMotor.setPower(-power);
        Bot.rlMotor.setPower(-power);
        Bot.rrMotor.setPower(-power);
    }

    protected void stopCreepDrive() {
        Bot.flMotor.setPower(0);
        Bot.frMotor.setPower(0);
        Bot.rlMotor.setPower(0);
        Bot.rrMotor.setPower(0);

    }
}







