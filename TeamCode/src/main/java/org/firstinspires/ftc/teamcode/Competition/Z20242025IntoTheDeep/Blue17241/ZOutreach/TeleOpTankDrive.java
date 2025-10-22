package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.ZOutreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.ZOutreach.IMBot.IMBot;


@Disabled
@TeleOp( name = "Tank Drive")

public class TeleOpTankDrive extends OpMode {

    public IMBot Bot = new IMBot();

    private float leftStickY1;
    private float rightStickY1;
    private float leftStickX1;
    private float rightStickX1;

    private float leftStickY2;
    private float leftStickX2;
    private float rightStickY2;
    private float rightStickX2;

    public float dpad_left;
    public float dpad_right;
    public float dpad_down;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;

    public double leftMotorValue;
    public double rightMotorValue;

    boolean tankDrive = true;
//    FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        Bot.stopMotors();

        leftStickY1 = 0;
        leftStickX1 = 0;
        rightStickY1 = 0;
        rightStickX1 = 0;

        leftStickY2 = 0;
        leftStickX2 = 0;
        rightStickY2 = 0;
        rightStickX2 = 0;
    }

    @Override
    public void loop() {
        getController();
        if (tankDrive == true) {
            tankDrive();
        } else {
            arcadeDrive();
        }

        driveSpeed();

    }

    public void driveSpeed() {
        if (gamepad1.dpad_up) {
            speedMultiply = 1;
        }

        if (gamepad1.dpad_down) {
            speedMultiply = 0.5;
        }
    }

    public void getController() {
        leftStickY1 = -gamepad1.left_stick_y;
        leftStickX1 = -gamepad1.left_stick_x;
        rightStickY1 = -gamepad1.right_stick_y;
        rightStickX1 = -gamepad1.right_stick_x;

        leftStickY2 = -gamepad2.left_stick_y;
        leftStickX2 = -gamepad2.left_stick_x;
        rightStickY2 = -gamepad2.right_stick_y;
        rightStickX2 = -gamepad2.right_stick_x;

        if (gamepad1.b) {
            tankDrive = true;
        } else if (gamepad1.x) {
            tankDrive = false;
        }
    }


    public void tankDrive() {
        double powerFLM = leftStickY1 * speedMultiply;
        Bot.frontRightMotor.setPower(powerFLM);
        double powerRLM = leftStickY1 * speedMultiply;
        Bot.rearRightMotor.setPower(powerRLM);
        double powerFRM = rightStickY1 * speedMultiply;
        Bot.frontLeftMotor.setPower(powerFRM);
        double powerRRM = rightStickY1 * speedMultiply;
        Bot.rearLeftMotor.setPower(powerRRM);
        telemetry.addData("powerFLM ", powerFLM);
        telemetry.addData("powerRLM ", powerRLM);
        telemetry.addData("MotopowerFLM ", powerFRM);
        telemetry.addData("MotopowerFLM ", powerRRM);
        telemetry.addData(" TANK DRIVE MODEL: ", tankDrive);
        telemetry.addData(" SPEED MODDE ", speedMultiply);
        telemetry.update();
    }


    public void arcadeDrive() {
        leftMotorValue = leftStickY1 - leftStickX1;
        rightMotorValue = leftStickY1 + leftStickX1;
        leftMotorValue = Range.clip(leftMotorValue, -1, 1);
        rightMotorValue = Range.clip(rightMotorValue, -1, 1);
        Bot.frontLeftMotor.setPower(leftMotorValue * speedMultiply);
        Bot.rearLeftMotor.setPower(leftMotorValue * speedMultiply);
        Bot.frontRightMotor.setPower(rightMotorValue * speedMultiply);
        Bot.rearRightMotor.setPower(rightMotorValue * speedMultiply);
    }

    public void showTelem () {
        telemetry.addData(" TANK DRIVE MODEL: ", tankDrive);
//        telemetry.addData("Front Left Motor", )
    }

}
