package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Audience;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Red Launch Park Audience Cam")
public class RedLaunchParkAudienceCam extends RedAlliance {

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

    //private Limelight3A limelight;

    private PathState pathState = PathState.READY;
    private LaunchStateOne launchStateOne = LaunchStateOne.READY;

    protected Timer creepTimer;
    protected static final double CREEP_POWER = 0.5;
    protected static final double CREEP_TIMEOUT_S = 3;

    private Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(270));
    private final Pose launch = new Pose(86, 12, Math.toRadians(248));
    private final Pose intake = new Pose(96, 36, Math.toRadians(0));
    private final Pose intakePickupEnd = new Pose(112, 36, Math.toRadians(0));
    private final Pose park = new Pose(108, 12, Math.toRadians(0));

    private Path launchOne;
    private PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    boolean scoringDone = false;

    //public int motifID;

//    double targetTX = 23;
//    double targetTA = 3;
//    double llTolerance = 1.25;

    private void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        intakePath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        intakePickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickupEnd))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickupEnd.getHeading())
                .setGlobalDeceleration()
                .build();
        launchTwoPath = follower.pathBuilder()
                .addPath(new BezierCurve(intakePickupEnd, launch))
                .setLinearHeadingInterpolation(intakePickupEnd.getHeading(), launch.getHeading())
                .build();
        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();
    }

    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        telemetry.setMsTransmissionInterval(11);
//        limelight.pipelineSwitch(0);
//        limelight.start();

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

        creepTimer = new Timer();
        creepTimer.resetTimer();

//        LLResult result = limelight.getLatestResult();
//        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
//        for(LLResultTypes.FiducialResult fr : fiducialResults){
//            if(fr.getFiducialId() == 21){
//                motifID = 21;
//                telemetry.addLine("21");
//                //telemetry.addData("FI: ", fr.getFiducialId());
//                telemetry.update();
//
//            }
//            if(fr.getFiducialId() == 22){
//                motifID = 22;
//                telemetry.addLine("22");
//                //telemetry.addData("FI: ", fr.getFiducialId());
//                telemetry.update();
//
//            }
//            if(fr.getFiducialId() == 23){
//                motifID = 23;
//                telemetry.addLine("23");
//                //telemetry.addData("FI: ", fr.getFiducialId());
//                telemetry.update();
//            }
//            else{
//                motifID = 21;
//                telemetry.addLine("None detected");
//                telemetry.update();
//            }
//        }
    }


    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        //limelight.start();
        pathState = PathState.DRIVETOLAUNCH;
        launchStateOne = LaunchStateOne.READY;
        launchZone = LaunchZone.NONE;
    }

    public enum LaunchStateOne {OUTTAKE, INTAKEONE, WAITONE, INTAKETWO, WAITTWO, INTAKETHREE, WAIT, READY, IDLE, OUTTAKEONE, OUTTAKETWO, OUTTAKETHREE}
    public enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE, PICKUP, LAUNCHPOSTWO, LAUNCHTWO, DECIDE, CREEP_INTAKE, PARK, READY, WAIT;}
    @Override
    public void loop() {

        autoPathing();
        automaticLaunchOne();



//        LLResult result = limelight.getLatestResult();
//        double txDifference = result.getTx() - targetTX;
//        double taDifference = result.getTy() - targetTA;


        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        // Feedback to Driver Hub for debugging
        telemetry.addData("launch state", launchStateOne);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Is Busy: ", follower.isBusy());
//        telemetry.addData("Tx Difference: ", txDifference);
//        telemetry.addData("Ta Difference: ", taDifference);
//        telemetry.addData("LL Ta:", result.getTa());
//        telemetry.addData("LL Tx: ", result.getTx());
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

//                if(!follower.isBusy()){
//                    autoPositioning();
//                }
//                if(targetFound == true) {
//                    pathState = PathState.LAUNCH;
//                    pathTimer.resetTimer();
//                }
                break;

            case LAUNCH:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchStateOne == LaunchStateOne.READY || launchStateOne == LaunchStateOne.IDLE) {
                        launchStateOne = LaunchStateOne.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    Bot.ballIntake();
                    follower.followPath(intakePath);
                    waitTimer.resetTimer();
                    pathState = PathState.INTAKE;
                    pathTimer.resetTimer();
                    Bot.artifactPushMiddle();
                    Bot.artifactPushUps();
                }
                break;
            case INTAKE:
                //Bot.artifactPushUps();
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3){
                    //Bot.artifactPushUps();
                    creepTimer.resetTimer();
                    scoringDone = false;
                    pathState = PathState.CREEP_INTAKE;
                    Bot.artifactPushUps();
                    pathTimer.resetTimer();
                }
                break;
            case CREEP_INTAKE:
                driveForwardCreep(CREEP_POWER);
                if (creepTimer.getElapsedTimeSeconds() >= CREEP_TIMEOUT_S) {
                    stopCreepDrive();
                    Bot.intakeStop();
                    waitTimer.resetTimer();

                    follower.followPath(launchTwoPath, true);
                    pathState = PathState.LAUNCHPOSTWO;  // continue your normal flow
                    pathTimer.resetTimer();
                }
                break;
            case LAUNCHPOSTWO:
                if(!follower.isBusy()){
                    pathState = PathState.READY;
                    pathTimer.resetTimer();
                    scoringDone = false;
                }
                break;
            case LAUNCHTWO:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchStateOne == LaunchStateOne.READY || launchStateOne == LaunchStateOne.IDLE) {
                        launchStateOne = LaunchStateOne.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    Bot.ballIntake();
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                    pathTimer.resetTimer();
                }
                break;

            case PARK:
                if(!follower.isBusy()) {
                    follower.followPath(parkPath);
                    pathState = PathState.READY;
                    pathTimer.resetTimer();
                }
//                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {
//                    pathState = PathState.DECIDE;
//                }
                break;
//            case DECIDE:
//                if (motifID == 21){
//                    telemetry.addLine("21");
//                    telemetry.update();
//                    pathState = PathState.READY;
//                }
//                if(motifID == 22){
//                    telemetry.addLine("22");
//                    telemetry.update();
//                    pathState = PathState.READY;
//                }
//                if(motifID == 23){
//                    telemetry.addLine("23");
//                    telemetry.update();
//                    pathState = PathState.READY;
//                }
            case READY:
                break;
        }
    }

    //LAUNCHING CODE DURING AUTO - MIGHT NEED TO BE CHANGED
    public void automaticLaunchOne() {
        switch(launchStateOne) {
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
                launchStateOne = LaunchStateOne.WAIT;
                break;
            case WAIT:
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    //Bot.intakeStop();
                    launchStateOne = LaunchStateOne.INTAKEONE;
                    //Bot.ballLaunchAutoBack();
                }
                break;
            case INTAKEONE:
                Bot.ballIntake();
                Bot.ballLaunchAutoBack();
                Bot.artifactPushDown();
                if (intakeTimer.getElapsedTimeSeconds() > 2) {
                    Bot.intakeStop();
                    Bot.artifactPushUps();
                    waitTimer.resetTimer();
                    launchStateOne = LaunchStateOne.IDLE;
                    scoringDone = true;

                }
                break;
//            case WAITONE:
//                if (waitTimer.getElapsedTimeSeconds() > 1) {
//                    intakeTimer.resetTimer();
//                    Bot.intakeStop();
//                    launchStateOne = LaunchStateOne.INTAKETWO;
//                }
//                break;
//            case INTAKETWO:
//                Bot.ballIntake();
//                if (intakeTimer.getElapsedTimeSeconds() > 1.5) {
//                    Bot.intakeStop();
//                    waitTimer.resetTimer();
//                    launchStateOne = LaunchStateOne.IDLE;
//                }
//                break;
            case IDLE:
                Bot.ballLaunchOne.setPower(0);
                Bot.ballLaunchTwo.setPower(0);
                //Bot.artifactPushUps();
                Bot.intakeStop();
                break;
        }
    }


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


