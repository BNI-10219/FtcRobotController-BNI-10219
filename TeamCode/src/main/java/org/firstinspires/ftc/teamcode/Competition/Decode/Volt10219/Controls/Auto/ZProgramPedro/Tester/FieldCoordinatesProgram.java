package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Tester;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Autonomous(name = "Field Coordinate Testing Program", group = "Examples")
public class FieldCoordinatesProgram extends AutoMainProgram {

    Follower follower;

    private Timer pathTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toDegrees(0));//0, 72
    private final Pose ytestPose = new Pose(90, 90, Math.toDegrees(90));//72, 72
    private final Pose xtestPose = new Pose(50, 40, Math.toDegrees(50));//72, 98

    private PathChain yTest, xTest;

    private void buildPaths(){
        yTest = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, ytestPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ytestPose.getHeading())
                .build();

        xTest = follower.pathBuilder()
                .addPath(new BezierCurve(ytestPose, xtestPose))
                .setLinearHeadingInterpolation(ytestPose.getHeading(), xtestPose.getHeading())
                .build();
    }


    private void pathStates(){
        switch(pathState){
            case 0:
                follower.followPath(yTest);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(xTest);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;
        }

    }

    private void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
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
        setPathState(0);
    }

    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
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
