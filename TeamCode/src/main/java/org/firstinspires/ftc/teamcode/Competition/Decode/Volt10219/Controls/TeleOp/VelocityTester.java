package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

//@Disabled
@TeleOp(name = "Velocity Tester", group="Tester")

public class VelocityTester extends OpMode {

//    public DecodeBot Bot = new DecodeBot();

    public DcMotorEx ballLaunchOne = null;
    public DcMotorEx ballLaunchTwo = null;
    public CRServo ballStop = null;
    public DcMotor ballIntake = null;


    public double velocity = 500;
    public double velocity_low = 200;
    public double velocity_med = 1200;
    public double velocity_high = 2300;
    public double incValue = 1;

    @Override
    public void init() {
//        Bot.initRobot(hardwareMap);

        ballStop = hardwareMap.get(CRServo.class, "ball_stop");
        ballStop.setDirection(CRServo.Direction.FORWARD);


        ballLaunchOne = hardwareMap.get(DcMotorEx.class, "ball_launch_one");
        ballLaunchOne.setDirection(DcMotorSimple.Direction.REVERSE);
        ballLaunchOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ballLaunchTwo = hardwareMap.get(DcMotorEx.class, "ball_launch_two");
        ballLaunchTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        ballLaunchTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ballIntake = hardwareMap.dcMotor.get("intake_one");
        ballIntake.setDirection(DcMotor.Direction.FORWARD);
        ballIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ballIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            velocity += incValue;
        }

        if (gamepad1.left_bumper) {
            velocity -= incValue;
        }


        //velocity = Range.clip(velocity, 0, 5000);
        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);
        update_telemetry();

        artifactPushControl();
        intakeControl();
    }

    public void artifactPushControl(){

        if(gamepad1.y){
            ballStop.setPower(-1);
        }
        if(gamepad1.a){
            ballStop.setPower(0);
        }

        if(gamepad1.dpad_up){
            ballIntake.setPower(-1);
        }
        if(gamepad1.dpad_down){
            ballIntake.setPower(0);
        }

    }

    public void intakeControl(){

    }


    public void update_telemetry () {
        telemetry.addData("left motor power: ", ballLaunchOne.getPower());
        telemetry.addData("left motor velocity: ", Math.abs(ballLaunchOne.getVelocity()));
        telemetry.addData("left motor encoders: ", ballLaunchOne.getCurrentPosition());
        telemetry.addLine("");
        telemetry.addData("Right motor power: ", ballLaunchTwo.getPower());
        telemetry.addData("Right motor velocity: ", Math.abs(ballLaunchTwo.getVelocity()));
        telemetry.addData("Right motor encoders: ", ballLaunchTwo.getCurrentPosition());
        telemetry.addData("Velocity: ", velocity);

    }

}
