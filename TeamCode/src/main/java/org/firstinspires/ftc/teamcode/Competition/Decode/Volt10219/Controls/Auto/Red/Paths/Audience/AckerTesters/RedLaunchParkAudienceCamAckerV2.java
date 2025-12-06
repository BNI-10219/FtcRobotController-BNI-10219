package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Audience.AckerTesters;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;
import java.util.List;

/*** This Version Uses Pedro for Intaking... which is not able to slow down ***/

@Disabled
@Autonomous(name = "Tester V2 Pedro: Red Launch Park Audience Cam")
public class RedLaunchParkAudienceCamAckerV2 extends RedAlliance {

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

    // Limelight and April Tag Variables
    protected Limelight3A limelight;
    protected int motifID;
    protected static final int PPG_TAG_ID = 23;
    protected static final int PGP_TAG_ID = 22;
    protected static final int GPP_TAG_ID = 21;

    // Pedro Pathing Follower
    protected Follower fastFollower;
    protected Follower slowFollower;
    protected Follower activeFollower;   // always points to whichever is currently controlling

    // State Machine for Launching
    protected enum LaunchState {OUTTAKE, WAIT, READY, IDLE}
    protected LaunchState launchState = LaunchState.READY;

    // State Machine for Auto Pathing
    protected enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE_START, PICKUP, INTAKE_PICKUP, LAUNCHPOSTWO, LAUNCHTWO, DECIDE, PARK, READY, WAIT;}
    protected PathState pathState = PathState.READY;


    //******** Timers and Counters and Boolean Controllers
    protected Timer creepTimer;
    protected Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;
    protected int shotCount = 0;
    protected boolean scoringDone = false;

    //********* Pedro Pathing Poses

    protected final Pose startPose = new Pose(96, 8, Math.toRadians(270));
    protected final Pose launch = new Pose(86, 12, Math.toRadians(248));
    protected final Pose park = new Pose(96, 24, Math.toRadians(0));

    // For Testing without Motif
    protected final Pose intake = new Pose(96, 34, Math.toRadians(0));
    protected final Pose intakePickupEnd = new Pose(112, 34, Math.toRadians(0));

    // Preparing for Intake Using Motifs... Not Used Yet.
    protected final Pose PPGPose = new Pose(96, 83, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose PPGPosePickup = new Pose(112, 83, Math.toRadians(0));

    protected final Pose PGPPose = new Pose(96, 59, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose PGPPosePickup = new Pose(112, 59, Math.toRadians(0));

    protected final Pose GPPPose = new Pose(96, 34, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    protected final Pose GPPPosePickup = new Pose(112, 34, Math.toRadians(0));


    //************ Building Paths for Pedro

    protected Path launchOne;
    protected PathChain intakePath, intakePickupPath, launchTwoPath, parkPath;

    // Preparing for Intaking Motifs... Not Used Yet
    protected PathChain moveToPPG, grabPPG, scorePPG;
    protected PathChain moveToPGP, grabPGP, scorePGP;
    protected PathChain moveToGPP, grabGPP, scoreGPP;
    protected PathChain chosenMoveToPath, chosenPickupPath, chosenScorePath;

    // Basic Path Builder (Laumch, Default Intake, Intake Pickup, Park)
    protected void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        intakePath = fastFollower.pathBuilder()
                .addPath(new BezierCurve(launch, intake))
                .setLinearHeadingInterpolation(launch.getHeading(), intake.getHeading())
                .build();
        // Note this path uses slow follower
        intakePickupPath = slowFollower.pathBuilder()
                .addPath(new BezierCurve(intake, intakePickupEnd))
                .setLinearHeadingInterpolation(intake.getHeading(), intakePickupEnd.getHeading())
                .setGlobalDeceleration()
                .build();
        launchTwoPath = fastFollower.pathBuilder()
                .addPath(new BezierCurve(intakePickupEnd, launch))
                .setLinearHeadingInterpolation(intakePickupEnd.getHeading(), launch.getHeading())
                .build();
        parkPath = fastFollower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();
    }

    // Build the PPG Pathing (this uses BezierLine instead of curve... change????)
    public void buildPathsPPG() {
        moveToPPG = fastFollower.pathBuilder() //
                .addPath(new BezierCurve(launch, PPGPose))
                .setLinearHeadingInterpolation(launch.getHeading(), PPGPose.getHeading())
                .build();
        grabPPG = slowFollower.pathBuilder()
                .addPath(new BezierLine(PPGPose, PPGPosePickup))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), PPGPosePickup.getHeading())
                .build();
        scorePPG = fastFollower.pathBuilder()
                .addPath(new BezierCurve(PPGPosePickup, launch))
                .setLinearHeadingInterpolation(PPGPosePickup.getHeading(), launch.getHeading())
                .build();
    }

    // Build the PGP Pathing (this uses BezierLine instead of curve... change????)
    public void buildPathsPGP() {
        moveToPGP = fastFollower.pathBuilder()
                .addPath(new BezierCurve(launch, PGPPose))
                .setLinearHeadingInterpolation(launch.getHeading(), PGPPose.getHeading())
                .build();
        grabPGP = slowFollower.pathBuilder()
                .addPath(new BezierLine(PGPPose, PGPPosePickup))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), PGPPosePickup.getHeading())
                .build();

        scorePGP = slowFollower.pathBuilder()
                .addPath(new BezierCurve(PGPPosePickup, launch))
                .setLinearHeadingInterpolation(PGPPosePickup.getHeading(), launch.getHeading())
                .build();
    }

    // Build the GPP Pathing (this uses BezierLine instead of curve... change????)
    public void buildPathsGPP() {
        moveToGPP = fastFollower.pathBuilder()
                .addPath(new BezierCurve(launch, GPPPose))
                .setLinearHeadingInterpolation(launch.getHeading(), GPPPose.getHeading())
                .build();
        grabGPP = slowFollower.pathBuilder()
                .addPath(new BezierLine(GPPPose, GPPPosePickup))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), GPPPosePickup.getHeading())
                .build();
        scoreGPP = fastFollower.pathBuilder()
                .addPath(new BezierCurve(GPPPosePickup, launch))
                .setLinearHeadingInterpolation(GPPPosePickup.getHeading(), launch.getHeading())
                .build();
    }


    //******** Op Modes ********
    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        initLimelight();

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

        // Run Limelight Detector Two Times in Init
        detectMotif();
        detectMotif();

        //Pedro Followers
        fastFollower = Constants.createFollower(hardwareMap);
        slowFollower = Constants.slowFollower(hardwareMap);
        activeFollower = fastFollower;

        // Build Pedro Paths
        buildPaths();
        buildPathsGPP();
        buildPathsPGP();
        buildPathsPPG();

        // Same Starting Pose for each follower
        fastFollower.setStartingPose(startPose);
        slowFollower.setStartingPose(startPose);
        fastFollower.getHeading();
        slowFollower.getHeading();

        creepTimer = new Timer();
        creepTimer.resetTimer();


    }


    public void init_loop() {}

    public void start() {
        // Run Limelight Detector One more Times ?
        detectMotif();
        opmodeTimer.resetTimer();
        pathState = PathState.DRIVETOLAUNCH;
        launchState = LaunchState.IDLE;
        launchZone = LaunchZone.NONE;
        scoringDone = false;
        shotCount = 0;
    }

    @Override
    public void loop() {

        autoPathing();
        automaticLaunch();
        activeFollower.update();
        telemetryUpdate();
    }

    public void stop() {}



    //**** Pathing State Machine **********
    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:
                useFastFollower();
                activeFollower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                launchState = LaunchState.READY;
                pathTimer.resetTimer();
                break;

            case LAUNCH:
                if (!activeFollower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                         launchState = LaunchState.OUTTAKE;
                    }
                }
                if (scoringDone) {
                    //Bot.artifactPushUps();
                    Bot.ballIntake();

                    // Path Detection using April Tag
                    if (motifID == PPG_TAG_ID) { chosenMoveToPath = moveToPPG;} // Path for 23
                    else if (motifID == PGP_TAG_ID) {chosenMoveToPath = moveToPGP;} // Path for 22
                    else { chosenMoveToPath = moveToGPP;}   // Path f
                    activeFollower.followPath(chosenMoveToPath);

                    // Transition to Next State
                    waitTimer.resetTimer();
                    pathState = PathState.INTAKE_START;
                    pathTimer.resetTimer();

                }
                break;

            case INTAKE_START:
                if (!activeFollower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    creepTimer.resetTimer();

                    // Change from Fast Follower to Slow Follower
                    useSlowFollower();

                    // Path Detection using April Tag
                    if (motifID == PPG_TAG_ID) { chosenPickupPath = grabPPG;} // Path for 23
                    else if (motifID == PGP_TAG_ID) {chosenPickupPath = grabPGP;} // Path for 22
                    else { chosenPickupPath = grabGPP;}   // Path for 21

                    // Transition to Next State
                    activeFollower.followPath(chosenPickupPath, true);
                    pathState = PathState.INTAKE_PICKUP;
                    Bot.ballIntake();
                    pathTimer.resetTimer();
                }
                break;

            case INTAKE_PICKUP:
                if (!activeFollower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4) {

                    // Change from Slow Follower to Fast Follower
                    useFastFollower();

                    // Path Detection using April Tag
                    if (motifID == PPG_TAG_ID) { chosenScorePath = scorePPG;} // Path for 23
                    else if (motifID == PGP_TAG_ID) {chosenScorePath = scorePGP;} // Path for 22
                    else { chosenScorePath = scoreGPP;}   // Path 21
                    activeFollower.followPath(chosenScorePath);

                    // Transition to Next State
                    pathState = PathState.LAUNCHPOSTWO;
                    launchState = LaunchState.READY;
                    scoringDone = false;
                    pathTimer.resetTimer();
                }
                break;


            case LAUNCHPOSTWO:
                if ( !activeFollower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                        Bot.intakeStop();
                        launchState = LaunchState.OUTTAKE;
                    }
                }

                if (scoringDone) {
                    //Bot.artifactPushUps();
                    Bot.ballIntake();

                    activeFollower.followPath(parkPath);
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                    pathTimer.resetTimer();

                }
                break;

            case PARK:
                if(!activeFollower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    pathState = PathState.READY;
                    pathTimer.resetTimer();
                    scoringDone = true;
                    launchState = LaunchState.IDLE;
                    Bot.intakeStop();

                }
                break;

            case READY:
                break;
        }
    }

    //**** Launching State Machine **********
    public void automaticLaunch() {
        switch(launchState) {

            case READY:
                Bot.ballLaunchBackField();
                outtakeTimer.resetTimer();
                intakeTimer.resetTimer();
                break;

            case OUTTAKE:
                Bot.ballLaunchAutoBackFirst();
                if (outtakeTimer.getElapsedTimeSeconds() > 2.5) {
                    //Bot.artifactPushDown();
                    waitTimer.resetTimer();
                    intakeTimer.resetTimer();
                    launchState = LaunchState.WAIT;
                    Bot.intakeStop();
                }
                break;

            case WAIT:
                Bot.ballIntake();//intake sooner
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    scoringDone = true;
                    shotCount ++;
                    launchState = LaunchState.IDLE;
                    //Bot.artifactPushUps();
                }
                break;

            case IDLE:
                Bot.ballLaunchOne.setPower(0);
                Bot.ballLaunchTwo.setPower(0);
                break;


        }
    }

    // Init Limelight
    public void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    // April Tag Motif Selection ID
    public void detectMotif() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        for(LLResultTypes.FiducialResult fr : fiducialResults){

           // GPP
            if(fr.getFiducialId() == 21) {
                motifID = 21;
                telemetry.addLine("21");
                telemetry.addData("FI: ", fr.getFiducialId());


            }
            // PGP Detection
            else if (fr.getFiducialId() == 22) {
                motifID = 22;
                telemetry.addLine("Detected PGP - 22");
                telemetry.addData("FI: ", fr.getFiducialId());

            }
            // PPG Detection
            else if (fr.getFiducialId() == 23) {
                motifID = 23;
                telemetry.addLine("Detected PPG - 23");
                telemetry.addData("FI: ", fr.getFiducialId());

            }
            // GPP Detection
            else if (fr.getFiducialId() == 21) {
                motifID = 21;
                telemetry.addLine("Detected GPP - 21");
                telemetry.addData("FI: ", fr.getFiducialId());

            }
            // Default to GPP
            else {
                motifID = 21;
                telemetry.addLine("None detected. Default to 21 GPP");
                telemetry.addData("FI: ", fr.getFiducialId());
                telemetry.update();
            }
        }
    }

    // Helper Methods to Manage Transition between Fast and Slow Followers
    public void useFastFollower() {
        if (activeFollower != null && activeFollower != fastFollower) {
            fastFollower.setPose(activeFollower.getPose());
        }
        activeFollower = fastFollower;
    }

    public void useSlowFollower() {
        if (activeFollower != null && activeFollower != slowFollower) {
            slowFollower.setPose(activeFollower.getPose());
        }
        activeFollower = slowFollower;
    }

    // Telemetry Update
    public void telemetryUpdate() {
        telemetry.addData("launch state", launchState);
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("x", activeFollower.getPose().getX());
        telemetry.addData("y", activeFollower.getPose().getY());
        telemetry.addData("heading", activeFollower.getPose().getHeading());
        telemetry.addData("Is Busy: ", activeFollower.isBusy());
        telemetry.addData("Motif ID: ", motifID);
        telemetry.update();
    }


}


