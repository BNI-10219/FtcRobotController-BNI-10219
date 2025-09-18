package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

public class TeleOp extends OpMode {
    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double powerThreshold = 0;
    double speedMultiply = 1;

    double flSpeed;
    double frSpeed;
    double rlSpeed;
    double rrSpeed;

    public DecodeBot Bot = new DecodeBot();

    @Override
    public void init(){
        Bot.initRobot(hardwareMap);
    }

    public void init_loop(){}

    public void start(){}

    @Override
    public void loop(){
        speedControl();
        drive();
        launcherControl();
        intakeControl();
        telemetryOutput();
    }

    public void drive(){

        leftStickYVal = -gamepad1.left_stick_y;
        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
        leftStickXVal = gamepad1.left_stick_x;
        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
        rightStickXVal = gamepad1.right_stick_x;
        rightStickXVal = Range.clip(rightStickXVal, -1, 1);

        flSpeed = leftStickYVal+leftStickXVal+rightStickXVal;
        flSpeed = Range.clip(flSpeed, -1, 1);
        frSpeed = leftStickYVal-leftStickXVal-rightStickXVal;
        flSpeed = Range.clip(flSpeed, -1, 1);
        rlSpeed = leftStickYVal-leftStickXVal+rightStickXVal;
        rlSpeed = Range.clip(rlSpeed, -1, 1);
        rrSpeed = leftStickYVal+leftStickXVal-rightStickXVal;
        rrSpeed = Range.clip(rrSpeed, -1, 1);

        if (flSpeed <= powerThreshold && flSpeed >= -powerThreshold){
            flSpeed = 0;
            Bot.flMotor.setPower(flSpeed);
        }else{
            Bot.flMotor.setPower(flSpeed * speedMultiply);
        }

        if(frSpeed <= powerThreshold && frSpeed >= -powerThreshold){
            frSpeed = 0;
            Bot.frMotor.setPower(frSpeed);
        }else{
            Bot.frMotor.setPower(frSpeed * speedMultiply);
        }

        if(rlSpeed <= powerThreshold && rlSpeed >= -powerThreshold) {
            rlSpeed = 0;
            Bot.rlMotor.setPower(rlSpeed);
        }else{
            Bot.rlMotor.setPower(rlSpeed * speedMultiply);
        }

        if(rrSpeed <= powerThreshold && rrSpeed >= -powerThreshold) {
            rrSpeed = 0;
            Bot.rrMotor.setPower(rrSpeed);
        }else{
            Bot.rrMotor.setPower(rrSpeed * speedMultiply);
        }
    }

    public void launcherControl(){

    }

    public void intakeControl(){

    }

    public void stateControl(){

    }

    public enum launchState{
        LAUNCH_START,

    }

    public void speedControl(){
        if(gamepad1.dpad_up){
            speedMultiply = 1;
        }
        else if(gamepad1.dpad_down){
            speedMultiply = .5;
        }

    }

    public void telemetryOutput(){
        telemetry.update();
    }
}
