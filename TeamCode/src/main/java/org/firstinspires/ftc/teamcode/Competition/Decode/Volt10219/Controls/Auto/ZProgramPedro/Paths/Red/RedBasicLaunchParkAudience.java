package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Paths.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.AutoMainProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Paths.Blue.BlueBasicLaunchParkBackstage;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.ZProgramPedro.Tester.AngleCoordinatesProgram;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.pedroPathing.Constants;

@Disabled
@Autonomous(name = "Red Basic Launch Park Audience", group = "Red")
public class RedBasicLaunchParkAudience extends AutoMainProgram {

    Follower follower;

    private PathState pathState = PathState.READY;

    private final Pose startPose = new Pose(96, 8, Math.toRadians(90));//test out angles
    private final Pose launch = new Pose(84, 84, Math.toRadians(225));
    private final Pose park = new Pose(96, 36, Math.toRadians(0));


    private Path launchPath;
    private PathChain parkPath;

    private void buildPaths(){
        launchPath = new Path(new BezierCurve(startPose, launch));
        launchPath.setLinearHeadingInterpolation(startPose.getHeading(), launch.getHeading());


        parkPath = follower.pathBuilder()
                .addPath(new BezierCurve(launch, park))
                .setLinearHeadingInterpolation(launch.getHeading(), park.getHeading())
                .build();
    }

    public enum PathState{
        LAUNCH,
        PARK,
        READY;
    }


    private void pathStates(){
        switch(pathState){
            case LAUNCH:
                follower.followPath(launchPath);
                pathState = PathState.PARK;
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

    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    public void init_loop() {}

    public void start() {
        pathState = PathState.LAUNCH;
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
