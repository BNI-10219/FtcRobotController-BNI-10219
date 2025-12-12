package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.Paths.Backstage;


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

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.Paths.Backstage.RedLaunchParkBackstageCamOliviaV3;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Red.RedAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

import java.util.List;


/**** This Version Uses Creep Foward Controller using Pedro Poses and Pinpoint for Slow Intake ***/

@Autonomous(name = "Blue Backstage - Cam, Complex")
public class BlueLaunchParkBackstageCamOliviaV3 extends BlueAlliance {

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
             // safety timeout


    // Pedro Pathing Follower
    protected Follower follower;

    // State Machine for Launching
    protected enum LaunchState {OUTTAKE, WAIT, READY, IDLE}
    protected LaunchState launchState = LaunchState.READY;

    // State Machine for Auto Pathing
    protected enum PathState{DRIVETOLAUNCH, LAUNCH, INTAKE_START, INTAKE_CREEP, WAITDETECT, DETECTMOTIF, LAUNCHPOSTWO, LAUNCHTWO, PARK, READY, WAIT;}
    protected PathState pathState = PathState.READY;


    //******** Timers and Counters and Boolean Controllers
    protected Timer creepTimer;
    protected Timer opmodeTimer, intakeTimer, waitTimer, pathTimer, outtakeTimer;
    protected int shotCount = 0;
    protected boolean scoringDone = false;

    //********* Pedro Pathing Poses

    private final Pose startPose = new Pose(24, 132, Math.toRadians(315));
    private final Pose launch = new Pose(50, 76, Math.toRadians(303));
    //protected final Pose park = new Pose(42, 130, Math.toRadians(180));//near back wall
    protected final Pose park = new Pose(24, 105, Math.toRadians(180));//near left wall
    protected final Pose detectMotif = new Pose(49, 73, Math.toRadians(260));

    protected final Pose PPGPose = new Pose(48, 81, Math.toRadians(270)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose PPGPosePickup = new Pose(32, 81, Math.toRadians(270));

    protected final Pose PGPPose = new Pose(48, 57.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose PGPPosePickup = new Pose(32, 57.5, Math.toRadians(180));

    protected final Pose GPPPose = new Pose(48, 34, Math.toRadians(175)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    protected final Pose GPPPosePickup = new Pose(32, 34, Math.toRadians(178));



    //************ Building Paths for Pedro

    protected Path launchOne;
    protected PathChain intakePath, intakePickupPath, launchTwoPath, parkPath, detectMotifPath;

    // Preparing for Intaking Motifs... Not Used Yet
    protected PathChain moveToPPG, grabPPG, scorePPG;
    protected PathChain moveToPGP, grabPGP, scorePGP;
    protected PathChain moveToGPP, grabGPP, scoreGPP;
    protected PathChain chosenMoveToPath, chosenScorePath;

    // Basic Path Builder (Laumch, Default Intake, Intake Pickup, Park)
    protected void buildPaths() {
        launchOne = new Path(new BezierCurve(startPose, launch));
        launchOne.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());

        detectMotifPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, detectMotif))
                .setLinearHeadingInterpolation(launch.getHeading(), detectMotif.getHeading())
                .build();

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

    // Build the PPG Pathing
    public void buildPathsPPG() {
        moveToPPG = follower.pathBuilder() //
                .addPath(new BezierCurve(launch, PPGPose))
                .setLinearHeadingInterpolation(launch.getHeading(), PPGPose.getHeading())
                .build();
        grabPPG = follower.pathBuilder()
                .addPath(new BezierLine(PPGPose, PPGPosePickup))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), PPGPosePickup.getHeading())
                .build();
        scorePPG = follower.pathBuilder()
                .addPath(new BezierCurve(PPGPosePickup, launch))
                .setLinearHeadingInterpolation(PPGPosePickup.getHeading(), launch.getHeading())
                .build();
    }

    // Build the PGP Pathing
    public void buildPathsPGP() {
        moveToPGP = follower.pathBuilder()
                .addPath(new BezierCurve(launch, PGPPose))
                .setLinearHeadingInterpolation(launch.getHeading(), PGPPose.getHeading())
                .build();
        grabPGP = follower.pathBuilder()
                .addPath(new BezierLine(PGPPose, PGPPosePickup))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), PGPPosePickup.getHeading())
                .build();

        scorePGP = follower.pathBuilder()
                .addPath(new BezierCurve(PGPPosePickup, launch))
                .setLinearHeadingInterpolation(PGPPosePickup.getHeading(), launch.getHeading())
                .build();
    }

    // Build the GPP Pathing
    public void buildPathsGPP() {
        moveToGPP = follower.pathBuilder()
                .addPath(new BezierCurve(launch, GPPPose))
                .setLinearHeadingInterpolation(launch.getHeading(), GPPPose.getHeading())
                .build();
        grabGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPPPose, GPPPosePickup))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), GPPPosePickup.getHeading())
                .build();
        scoreGPP = follower.pathBuilder()
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
        follower = Constants.createFollower(hardwareMap);

        // Build Pedro Paths
        buildPaths();
        buildPathsGPP();
        buildPathsPGP();
        buildPathsPPG();

        // Same Starting Pose for each follower
        follower.setStartingPose(startPose);
        follower.getHeading();


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
        follower.update();
        // Creep Forward,  Override drive motors, and Check Status
        creepControl();

        telemetryUpdate();
    }

    public void stop() {}



    //**** Pathing State Machine **********
    public void autoPathing() {
        switch(pathState){
            case DRIVETOLAUNCH:

                follower.followPath(launchOne, true);
                pathState = PathState.LAUNCH;
                launchState = LaunchState.READY;
                pathTimer.resetTimer();
                break;

            case LAUNCH:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                         launchState = LaunchState.OUTTAKE;
                         pathTimer.resetTimer();
                    }
                }
                if (scoringDone) {
                    //Bot.artifactPushUps();
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        waitTimer.resetTimer();
                        follower.followPath(detectMotifPath);
                        pathState = PathState.DETECTMOTIF;
                        pathTimer.resetTimer();
                    }

                }
                break;
            case DETECTMOTIF:
                if(!follower.isBusy()|| pathTimer.getElapsedTimeSeconds() > 3){
                    pathState = PathState.WAITDETECT;
                    waitTimer.resetTimer();
                }
                break;
            case WAITDETECT:
                detectMotif();
                if(waitTimer.getElapsedTimeSeconds() > 2.5) {
                    if (motifID == PPG_TAG_ID) {
                        chosenMoveToPath = moveToPPG;
                    } // Path for 23
                    else if (motifID == PGP_TAG_ID) {
                        chosenMoveToPath = moveToPGP;
                    } // Path for 22
                    else {
                        chosenMoveToPath = moveToGPP;
                    }   // Path for 21
                    follower.followPath(chosenMoveToPath);
                    pathState = PathState.INTAKE_START;
                    pathTimer.resetTimer();
                }
                break;

            case INTAKE_START:
                Bot.ballIntake();
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    creepTimer.resetTimer();

                    // Transition to Creep Forward Control and State
                    startPinpointCreep();
                    pathState = PathState.INTAKE_CREEP;
                    Bot.ballIntake();
                    pathTimer.resetTimer();
                    scoringDone = false;
                }
                break;

            case INTAKE_CREEP:
                // Creep Forwards Controlled by outside of state machine for looping
                // State Transition happens in creepController()
                break;

            case LAUNCHPOSTWO:
                if ( !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    if (launchState == LaunchState.READY && !scoringDone ) {
                        Bot.intakeStop();
                        launchState = LaunchState.OUTTAKE;
                    }
                }

                if (scoringDone) {
                   // Bot.artifactPushUps();
                    Bot.ballIntake();

                    follower.followPath(parkPath);
                    waitTimer.resetTimer();
                    pathState = PathState.PARK;
                    pathTimer.resetTimer();

                }
                break;

            case PARK:
                if(!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
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
                Bot.ballLaunchMidV();
                outtakeTimer.resetTimer();
                intakeTimer.resetTimer();
                break;

            case OUTTAKE:
                Bot.ballIntake();
                outtakeTimer.resetTimer();

                //To change the velocity, change the numbers below
                Bot.ballLaunchBackField();//VELOCITY for launching 2nd artifact
                // Command + B to change the velocity(while the white line index thing is in the method)


//                Bot.artifactPushAuto();
//                Bot.artifactPushDown();
                Bot.intakeHoldStart();
                waitTimer.resetTimer();
                intakeTimer.resetTimer();
                launchState = LaunchState.WAIT;
                break;
//                Bot.ballLaunchAutoBack();
//                if (outtakeTimer.getElapsedTimeSeconds() > 2.5) {
//                    Bot.artifactPushDown();
//                    waitTimer.resetTimer();
//                    intakeTimer.resetTimer();
//                    launchState = LaunchState.WAIT;
//                    Bot.intakeStop();
//                }
//                break;

            case WAIT:
                Bot.ballIntake();//intake sooner
                if (waitTimer.getElapsedTimeSeconds() > 1) {
                    intakeTimer.resetTimer();
                    scoringDone = true;
                    shotCount ++;
                    launchState = LaunchState.IDLE;
                    Bot.intakeHoldStop();
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
                motifID = 23;
                telemetry.addLine("21");
                telemetry.addData("FI: ", fr.getFiducialId());


            }
            // PGP Detection
            else if (fr.getFiducialId() == 22) {
                motifID = 23;
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
                motifID = 23;
                telemetry.addLine("Detected GPP - 21");
                telemetry.addData("FI: ", fr.getFiducialId());

            }
            // Default to GPP
            else {
                motifID = 23;
                telemetry.addLine("None detected. Default to 21 GPP");
                telemetry.addData("FI: ", fr.getFiducialId());
                telemetry.update();
            }
        }
    }

    // ********  Creep Forward Helper Methods
    protected double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    // Starts the Creeper Slow Intake
    protected void startPinpointCreep() {
        Pose p = follower.getPose();
        creepStartPose = new Pose(p.getX(), p.getY(), p.getHeading());

        // We want to drive forward while holding:
        // - the current Y (no sideways drift)
        // - the current heading (no rotate)
        creepTargetY = p.getY();
        creepTargetHeading = p.getHeading();

        creepTimer.resetTimer();
    }

    protected void creepControl() {
        if (pathState == PathState.INTAKE_CREEP) {
            boolean creepDone = runPinpointCreepStep();

            if (creepDone) {

                // Pick the correct scoring path based on motif
                if (motifID == PPG_TAG_ID) { chosenScorePath = scorePPG;
                } else if (motifID == PGP_TAG_ID) { chosenScorePath = scorePGP;
                } else {chosenScorePath = scoreGPP; }

                follower.followPath(chosenScorePath);
                pathState = PathState.LAUNCHPOSTWO;
                launchState = LaunchState.READY;
                scoringDone = false;
                pathTimer.resetTimer();
            }
        }
    }

    protected boolean runPinpointCreepStep() {
        Pose curr = follower.getPose();

        double dx = curr.getX() - creepStartPose.getX();
        double dy = curr.getY() - creepStartPose.getY();

        double forwardProgress =
                dx * Math.cos(creepTargetHeading) +
                        dy * Math.sin(creepTargetHeading);

        double yError = curr.getY() - creepTargetY;
        double headingError = normalizeAngle(creepTargetHeading - curr.getHeading());

        double forward = creepForwardPower;
        double strafe  = -creepLatKp * yError;
        double turn    = -creepHeadingKp * headingError;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double rl = forward - strafe + turn;
        double rr = forward + strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(rl), Math.abs(rr)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            rl /= max;
            rr /= max;
        }

        Bot.flMotor.setPower(fl);
        Bot.frMotor.setPower(fr);
        Bot.rlMotor.setPower(rl);
        Bot.rrMotor.setPower(rr);

        boolean distanceDone = Math.abs(forwardProgress) >= creepTargetDistanceIn;
        boolean timeDone = creepTimer.getElapsedTimeSeconds() > creepTimeoutS;

        if (distanceDone || timeDone) {
            // Stop motors
            Bot.flMotor.setPower(0);
            Bot.frMotor.setPower(0);
            Bot.rlMotor.setPower(0);
            Bot.rrMotor.setPower(0);
            return true;    // tell caller method we are done creeping
        }

        return false;       // still creeping
    }

    // Telemetry Update
    public void telemetryUpdate() {
        telemetry.addData("launch state", launchState);
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Is Busy: ", follower.isBusy());
        telemetry.addData("Motif ID: ", motifID);
        telemetry.update();
    }


}


