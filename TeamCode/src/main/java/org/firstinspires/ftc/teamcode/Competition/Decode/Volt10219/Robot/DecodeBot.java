package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.DriveTrain.Mecanum;

public class DecodeBot extends Mecanum {
    public HardwareMap hwBot = null;

    //don't change "velocity"
    public double velocity = 1230;
    public double velocity_low = 1000;
    public double velocity_med = 1230; //936   d//1025
    public double velocity_high = 1400; //2300//1016 d//1050
    public Servo LED;


    public DecodeBot() {
    }
    public void LEDCon(int color) {
        float n = new float []{0, 0.279f, 0.333f, 0.388f, 0.5f, 0.611f, 0.722f}[color];
        LED.setPosition(n);
    }
    public void initRobot(HardwareMap hwBot) {

        flMotor = hwBot.dcMotor.get("front_Left_motor"); // CH Port 0
        frMotor = hwBot.dcMotor.get("front_right_motor"); // CH Port 1
        rlMotor = hwBot.dcMotor.get("rear_left_motor"); // CH Port 2
        rrMotor = hwBot.dcMotor.get("rear_right_motor"); // CH Port 3

        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        rlMotor.setDirection(DcMotor.Direction.REVERSE);
        rrMotor.setDirection(DcMotor.Direction.FORWARD);

        setMotorRunModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rrMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ballIntake = hwBot.dcMotor.get("intake_one");
        ballIntake.setDirection(DcMotor.Direction.FORWARD);
        ballIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ballIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ballIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        ballStop = hwBot.get(CRServo.class, "ball_stop");
        ballStop.setDirection(CRServo.Direction.FORWARD);

        LED = hwBot.servo.get("LED");

        ballLaunchOne = hwBot.get(DcMotorEx.class, "ball_launch_one");
        ballLaunchOne.setDirection(DcMotorSimple.Direction.REVERSE);
        ballLaunchOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ballLaunchTwo = hwBot.get(DcMotorEx.class, "ball_launch_two");
        ballLaunchTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        ballLaunchTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ballLaunchTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //IMU for Control Hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hwBot.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //init mechanisms
    }

    public void ballIntake() {
        //ballIntake.setPower(0.5);

        ballIntake.setPower(-1);
    }

    public void ballIntakeAuto(){
        ballIntake.setPower(-0.75);
    }

    public void ballOuttake() {
        ballIntake.setPower(1);
    }

    public void intakeStop(){
        ballIntake.setPower(0);
    }

    public void intakeHoldStop(){
        ballStop.setPower(0);
    }
    public void intakeHoldStart(){
        ballStop.setPower(-1);
    }
    public void intakeHoldReverse(){
        ballStop.setPower(1);
    }

//right bumper
    public void ballLaunchV() {
        //velocity = velocity_med;
        //velocity = Range.clip(velocity, 0, 5000);

        ballLaunchOne.setVelocity(velocity);
        ballLaunchTwo.setVelocity(velocity);

//        ballLaunchTwo.setPower(0.425);
//        ballLaunchOne.setPower(0.425);
    }

    public void ballLaunchAutoV(){
        ballLaunchOne.setVelocity(1165);
        ballLaunchTwo.setVelocity(1165);
    }
    public void ballLaunchAutoBackFirst(){
        ballLaunchOne.setVelocity(1120);//1120
        ballLaunchTwo.setVelocity(1120);//1120
    }
    public void ballLaunchAutoBackSecond(){
        ballLaunchOne.setVelocity(1215);
        ballLaunchTwo.setVelocity(1215);
    }

    //
    public void ballLaunchMidV(){
//        velocity = velocity_low;
//        velocity = Range.clip(velocity, 0, 1500);
        ballLaunchOne.setVelocity(velocity_med);
        ballLaunchTwo.setVelocity(velocity_med);

//        ballLaunchTwo.setPower(0.225);
//        ballLaunchOne.setPower(0.225);
    }

    //
    public void ballLaunchBackField(){
        //ballLaunchOne.setVelocity(1051);
        //ballLaunchTwo.setVelocity(1051);
//        velocity = velocity_high;
//        velocity = Range.clip(velocity, 0, 1500);
        ballLaunchOne.setVelocity(velocity_high);
        ballLaunchTwo.setVelocity(velocity_high);

//        ballLaunchTwo.setPower(0.625);
//        ballLaunchOne.setPower(0.625);
    }

    //left bumper
    public void ballLaunchFrontField() {
        ballLaunchOne.setVelocity(velocity_low);
        ballLaunchTwo.setVelocity(velocity_low);
    }

    public void ballLaunchStop(){
        ballLaunchTwo.setPower(0);
        ballLaunchOne.setPower(0);
    }



}
