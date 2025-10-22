package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Controls.Tester;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Robots.ProgrammerBot;

@Disabled
@TeleOp (name = "Tester:Driver Practice", group = "Testers")
public class TesterBasicTeleOp extends OpMode {

    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold;
    double speedMultiply;

    private static final int PROFILE_1 = 1;  //Charlie
    private static final int PROFILE_2 = 2;  // Evan
    private int currentProfile = PROFILE_2;
    //public double mechanismPower = ___;

    public ProgrammerBot Bot = new ProgrammerBot();

    @Override
    public void init() {

        Bot.initRobot(hardwareMap);
    }

    public void init_loop() {
    }

    public void start() {
    }

    @Override
    public void loop() {
        changeDriverProfile();
        speedControl();
        //drive();
        telemetryOutput();
        fieldCentricDrive();
    }

    public void changeDriverProfile() {
        if (gamepad1.left_bumper) {
            currentProfile = PROFILE_1;
        } else if (gamepad1.right_bumper) {
            currentProfile = PROFILE_2;
        }

    }

//    public void drive() {
//
//        // Joystick values
//        leftStickYVal = -gamepad1.left_stick_y;
//        leftStickYVal = Range.clip(leftStickYVal, -1, 1);
//        //double rightStickYVal = gamepad1.right_stick_y;
//        //rightStickYVal = Range.clip(rightStickYVal, -1, 1);
//
//        leftStickXVal = gamepad1.left_stick_x;
//        leftStickXVal = Range.clip(leftStickXVal, -1, 1);
//        rightStickXVal = gamepad1.right_stick_x;
//        rightStickXVal = Range.clip(rightStickXVal, -1, 1);
//
//        switch (currentProfile) {
//
//            // Name of Driver using Profile 1
//            case PROFILE_1:
//                // leftStickXVal controls rotation, and rightStickXVal controls strafing.
//                frontLeftSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
//                frontRightSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
//                rearLeftSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
//                rearRightSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
//                break;
//            // Name of Driver using Profile 2
//            case PROFILE_2:
//                //leftStickXVal controls strafing, and rightStickXVal controls rotation.
//                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
//                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
//                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
//                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
//                break;
//
//            // Default Driver Profile
//            default:
//                frontLeftSpeed = 0;
//                frontRightSpeed = 0;
//                rearLeftSpeed = 0;
//                rearRightSpeed = 0;
//                break;
//        }
//
//        // Clipping motor speeds to [-1, 1]
//        frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);
//        frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);
//        rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);
//        rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);
//
//        // Setting motor powers (with threshold check)
//        setMotorPower(Bot.frontLeftMotor, frontLeftSpeed, powerThreshold, speedMultiply);
//        setMotorPower(Bot.frontRightMotor, frontRightSpeed, powerThreshold, speedMultiply);
//        setMotorPower(Bot.rearLeftMotor, rearLeftSpeed, powerThreshold, speedMultiply);
//        setMotorPower(Bot.rearRightMotor, rearRightSpeed, powerThreshold, speedMultiply);
//    }

    public void fieldCentricDrive(){
        if (gamepad1.options) {
            Bot.imu.resetYaw();
        }
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            Bot.imu.resetYaw();
        }

        double botHeading = Bot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        Bot.frontLeftMotor.setPower(frontLeftPower);
        Bot.rearLeftMotor.setPower(backLeftPower);
        Bot.frontRightMotor.setPower(frontRightPower);
        Bot.rearRightMotor.setPower(backRightPower);
    }

    //    public void fieldCentricDrivePinpoint(){
//        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//        double x = gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        // This button choice was made so that it is hard to hit on accident,
//        // it can be freely changed based on preference.
//        // The equivalent button is start on Xbox-style controllers.
//        if (gamepad1.y) {
//            resetHeading();
//            getHeading();
//            //ITDBot.imu.resetHeading();
//        }
//
//        double botHeading = getHeading();
//        //double botHeading = odo.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        ITDBot.frontLeftMotor.setPower(frontLeftPower);
//        ITDBot.rearLeftMotor.setPower(backLeftPower);
//        ITDBot.frontRightMotor.setPower(frontRightPower);
//        ITDBot.rearRightMotor.setPower(backRightPower);
//    }

    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

    public void telemetryOutput() {
        telemetry.addData("pwr ", "FL motor ", +frontLeftSpeed);
        telemetry.addData("pwr ", "FR motor ", +frontRightSpeed);
        telemetry.addData("pwr ", "RL motor ", +rearLeftSpeed);
        telemetry.addData("pwr ", "RR motor ", +rearRightSpeed);
        telemetry.update();
    }

    public void speedControl() {
        if (gamepad1.dpad_up) {
            speedMultiply = 0.5;
        } else if (gamepad1.dpad_right) {
            speedMultiply = 0.75;
        } else if (gamepad1.dpad_down) {
            speedMultiply = 0.25;
        } else if (gamepad1.dpad_left) {
            speedMultiply = 1;
        } else {
            speedMultiply = 1;
        }
    }


}