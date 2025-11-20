package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.BasicPathsFieldSetup;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Field Coordinate Testing", group = "Examples")
public class FieldCoordinates extends BlueAlliance {

    Follower follower;
    Follower slowFollower;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    public final Pose startPose = new Pose(100, 8, Math.toRadians(90));     // Red Far Launch Zone start
    public final Pose ytestPose = new Pose(72, 72, Math.toRadians(45));    // Red goal scoring pose
    public final Pose xtestPose = new Pose(120, 48, Math.toRadians(0));

    private Path ytest;
    private PathChain xTest;

    public void buildPaths(){
        ytest = new Path(new BezierCurve(startPose, ytestPose));
        ytest.setLinearHeadingInterpolation(startPose.getHeading(), ytestPose.getHeading());


        xTest = slowFollower.pathBuilder()
                .addPath(new BezierCurve(ytestPose, xtestPose))
                .setLinearHeadingInterpolation(ytestPose.getHeading(), xtestPose.getHeading())
                .build();
    }


    public void pathStates(){
        switch(pathState){
            case 0:
                follower.followPath(ytest);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    slowFollower.followPath(xTest);
                    setPathState(2);
                    break;
                    // Removed Break
                }
                //break;
            case 2:
                break;
        }

    }

    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        slowFollower = Constants.slowFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        slowFollower.setStartingPose(startPose);
    }

    public void init_loop() {}

    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        slowFollower.update();
        pathStates();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {}
}
