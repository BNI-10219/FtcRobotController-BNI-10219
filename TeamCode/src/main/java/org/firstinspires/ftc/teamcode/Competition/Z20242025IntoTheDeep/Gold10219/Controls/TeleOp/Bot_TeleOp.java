package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.TeleOp;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BrightLights.RGBIndicator;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Outgrabber.Outgrabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.SecondaryArm.SecondaryArm;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBot;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp(name = "A - Into the Deep", group = "competition")
public class Bot_TeleOp extends OpMode {

    boolean usePowers = true;
    double d1Power = 2;

    double leftStickXVal;
    double leftStickYVal;
    double rightStickXVal;

    double frontLeftSpeed;
    double frontRightSpeed;
    double rearLeftSpeed;
    double rearRightSpeed;

    double powerThreshold = 0;
    double speedMultiply = 1;
    boolean armSpeedMultiplier = false;

    public CompBot Bot = new CompBot();

    IMU imu = null;

    Grabber grabber = new Grabber();

    PrimaryArm primaryArm = new PrimaryArm();

    Outgrabber outgrabber = new Outgrabber();

    SecondaryArm secondaryArm = new SecondaryArm();
    RGBIndicator indicator = new RGBIndicator();

//    private Pose startPose;
//
//    private Follower follower;

    ElapsedTime primaryTimer = new ElapsedTime();
    ElapsedTime secondaryTimer = new ElapsedTime();
    ElapsedTime hookShortcutTimer = new ElapsedTime();
    ElapsedTime totalTimer = new ElapsedTime();

    public void init() {
        Bot.initRobot(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        telemetry.addLine("IMU Initialized");

        primaryTimer.reset();

//        pinpoint.setOp(this);
//        pinpoint.initPinpoint(hardwareMap);
//
//        telemetry.addLine("Pinpoint Initialized");
//
//        vision.setOp(this);
//        vision.initVision(hardwareMap, pinpoint);
//
//        telemetry.addLine("Vision Initialized");
//
//        pose.setOp(this);
//        pose.setDevices(vision, pinpoint);
//
//        telemetry.addLine("Pose Initiated");
//
//        vision.start();
//
//        telemetry.addLine("Vision Started");
//
//        pose.updateLLUsage(false);
//
//        Pose2D currentPose = pose.getSmartPose(PoseHelper.Alliances.BLUE);
//        telemetry.addData("Pose X: ", currentPose.getX(DistanceUnit.INCH));
//        telemetry.addData("Pose Y: ", currentPose.getY(DistanceUnit.INCH));
//        telemetry.addData("Pose H: ", currentPose.getHeading(AngleUnit.DEGREES));
//
//        startPose = new Pose(
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.RADIANS)
//        );
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        telemetry.addLine("Follower Initialized");

        primaryArm.initPrimaryArm(hardwareMap, Bot.LinearOp);
        grabber.initGrabber(hardwareMap);

        secondaryArm.initSecondaryArm(hardwareMap, Bot.LinearOp);
        outgrabber.initOutgrabber(hardwareMap);

        indicator.initIndicator(hardwareMap);

        telemetry.update();
    }

    public void start() {
        grabber.grab();
        grabber.headStraight();
        primaryArm.setRetract();
        secondaryArm.setRetract();
        totalTimer.reset();
    }

    public void loop() {
        speedControl();
        primaryDriverProfileSwitcher();
        secondaryDriverProfileSwitcher();
        drive();
        primaryShortcutChecker();
        secondaryShortcutChecker();
        shortcuts();
        mechanismsControl();
        indicatorCheck();
        primaryArm.rotationChecker();
        grabber.imuGyroCheck();
        grabber.tiltStateCheck();
        telemetryOutput();
    }

    public void speedControl() {
        if (gamepad1.left_trigger > 0.35) {
            speedMultiply = 0.3;
        } else {
            speedMultiply = 1;
        }

        if (gamepad2.left_stick_button) {
            armSpeedMultiplier = true;
        } else {
            armSpeedMultiplier = false;
        }
    }

    private enum driverProfiles {
        CLASSIC, FIELD_CENTRIC
    }

    private driverProfiles driverProfile = driverProfiles.CLASSIC;

    public void primaryDriverProfileSwitcher() {
        if (gamepad1.a) driverProfile = driverProfiles.CLASSIC;
        else if (gamepad1.b) driverProfile = driverProfiles.FIELD_CENTRIC;
    }

    public void secondaryDriverProfileSwitcher() {
        if (gamepad2.right_bumper) secondaryDriverTwo = true;
        else secondaryDriverTwo = false;
    }

    public void drive() {
        switch (driverProfile) {
            case CLASSIC:
                leftStickXVal = gamepad1.left_stick_x;
                leftStickXVal = Range.clip(leftStickXVal, -1, 1);

                leftStickYVal = -gamepad1.left_stick_y;
                leftStickYVal = Range.clip(leftStickYVal, -1, 1);

                rightStickXVal = gamepad1.right_stick_x;
                rightStickXVal = Range.clip(rightStickXVal, -1, 1);

                frontLeftSpeed = leftStickYVal + leftStickXVal + rightStickXVal;
                frontLeftSpeed = Range.clip(frontLeftSpeed, -1, 1);

                frontRightSpeed = leftStickYVal - leftStickXVal - rightStickXVal;
                frontRightSpeed = Range.clip(frontRightSpeed, -1, 1);

                rearLeftSpeed = leftStickYVal - leftStickXVal + rightStickXVal;
                rearLeftSpeed = Range.clip(rearLeftSpeed, -1, 1);

                rearRightSpeed = leftStickYVal + leftStickXVal - rightStickXVal;
                rearRightSpeed = Range.clip(rearRightSpeed, -1, 1);

                if (frontLeftSpeed <= powerThreshold && frontLeftSpeed >= -powerThreshold) {
                    frontLeftSpeed = 0;
                    Bot.frontLeftMotor.setPower(frontLeftSpeed);
                } else {
                    Bot.frontLeftMotor.setPower(frontLeftSpeed * speedMultiply);
                }

                if (frontRightSpeed <= powerThreshold && frontRightSpeed >= -powerThreshold) {
                    frontRightSpeed = 0;
                    Bot.frontRightMotor.setPower(frontRightSpeed);
                } else {
                    Bot.frontRightMotor.setPower(frontRightSpeed * speedMultiply);
                }

                if (rearLeftSpeed <= powerThreshold && rearLeftSpeed >= -powerThreshold) {
                    rearLeftSpeed = 0;
                    Bot.rearLeftMotor.setPower(rearLeftSpeed);
                } else {
                    Bot.rearLeftMotor.setPower(rearLeftSpeed * speedMultiply);
                }

                if (rearRightSpeed <= powerThreshold && rearRightSpeed >= -powerThreshold) {
                    rearRightSpeed = 0;
                    Bot.rearRightMotor.setPower(rearRightSpeed);
                } else {
                    Bot.rearRightMotor.setPower(rearRightSpeed * speedMultiply);
                }
                break;
            case FIELD_CENTRIC:
                double y, x, rx;
                if (usePowers) {
                    y = Math.abs(Math.pow(gamepad1.left_stick_y, d1Power)); // Remember, Y stick value is reversed
                    if (gamepad1.left_stick_y > 0) y *= -1;
                    x = Math.abs(Math.pow(gamepad1.left_stick_x, d1Power));
                    if (gamepad1.left_stick_x < 0) x *= -1;
                    rx = Math.abs(Math.pow(gamepad1.right_stick_x, d1Power));
                    if (gamepad1.right_stick_x < 0) rx *= -1;
                } else {
                    y = -gamepad1.left_stick_y;
                    x = gamepad1.left_stick_x;
                    rx = gamepad1.right_stick_x;
                }

                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

                if (usePowers) {
                    Bot.frontLeftMotor.setPower(frontLeftPower);
                    Bot.rearLeftMotor.setPower(backLeftPower);
                    Bot.frontRightMotor.setPower(frontRightPower);
                    Bot.rearRightMotor.setPower(backRightPower);
                } else {
                    Bot.frontLeftMotor.setPower(frontLeftPower * speedMultiply);
                    Bot.rearLeftMotor.setPower(backLeftPower * speedMultiply);
                    Bot.frontRightMotor.setPower(frontRightPower * speedMultiply);
                    Bot.rearRightMotor.setPower(backRightPower * speedMultiply);
                }

                break;
        }
    }

    private primaryShortcutCases primaryShortcutCase = primaryShortcutCases.NONE;

    private enum primaryShortcutCases {
        NONE, GRAB_SPECIMEN, HOOK_SPECIMEN, RETRACT_FROM_CHAMBER
    }

    private secondaryShortcutCases secondaryShortcutCase = secondaryShortcutCases.NONE;

    private enum secondaryShortcutCases {
        NONE, GRAB_SAMPLE
    }

    private grabSampleCases grabSampleCase = grabSampleCases.OPEN;

    private enum grabSampleCases {
        OPEN,
        TIMEOUT1,
        UPANDCENTER,
        RETRACT
    }

    private grabSpecimenCases grabSpecimenCase = grabSpecimenCases.CLOSE;

    private enum grabSpecimenCases {
        CLOSE,
        TIMEOUT1,
        UP,
        TIMEOUT2,
        RETRACT
    }

    private hookSpecimenCases hookSpecimenCase = hookSpecimenCases.HOOK;

    private enum hookSpecimenCases {
        HOOK,
        TIMEOUT1,
        DOWN
    }

    private retractFromChamberCases retractFromChamberCase = retractFromChamberCases.OPEN;

    private enum retractFromChamberCases {
        OPEN,
        TIMEOUT1,
        UP,
        TIMEOUT2,
        RETRACT,
        TUCK
    }

    public void primaryShortcutChecker() {
        switch (primaryShortcutCase) {
            case GRAB_SPECIMEN:
                grabSpecimen();
                break;
            case HOOK_SPECIMEN:
                hookSpecimen();
                break;
            case RETRACT_FROM_CHAMBER:
                retractFromChamber();
                break;
        }
    }

    public void secondaryShortcutChecker() {
        switch (secondaryShortcutCase) {
            case GRAB_SAMPLE:
                grabSample();
                break;
        }
    }

    public void grabSpecimen() {
        switch (grabSpecimenCase) {
            case CLOSE:
                grabber.grab();
                primaryTimer.reset();
                grabSpecimenCase = grabSpecimenCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (primaryTimer.time() > .5) {
                    grabSpecimenCase = grabSpecimenCases.UP;
                }
                break;
            case UP:
                primaryArm.setRotation(PrimaryArm.rotationStates.UP, 2, true);
                primaryTimer.reset();
                grabSpecimenCase = grabSpecimenCases.TIMEOUT2;
                break;
            case TIMEOUT2:
                if (primaryTimer.time() > .5) {
                    grabSpecimenCase = grabSpecimenCases.RETRACT;
                }
                break;
            case RETRACT:
                primaryArm.setRetract();
                primaryShortcutCase = primaryShortcutCases.NONE;
                grabSpecimenCase = grabSpecimenCases.CLOSE;
                break;
        }
    }

    public void grabSample() {
        switch (grabSampleCase) {
            case OPEN:
                outgrabber.grab();
                secondaryTimer.reset();
                grabSampleCase = grabSampleCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (secondaryTimer.time() > .25) {
                    grabSampleCase = grabSampleCases.UPANDCENTER;
                }
                break;
            case UPANDCENTER:
                outgrabber.upPosition();
                outgrabber.headStraight();
                secondaryTimer.reset();
                grabSampleCase = grabSampleCases.RETRACT;
                break;
            case RETRACT:
                secondaryArm.setRetract();
                secondaryShortcutCase = secondaryShortcutCases.NONE;
                grabSampleCase = grabSampleCases.OPEN;
                break;
        }
    }

    public void hookSpecimen() {
        switch (hookSpecimenCase) {
            case HOOK:
                grabber.setGrabberState(Grabber.grabberStates.HOOK);
                primaryTimer.reset();
                hookSpecimenCase = hookSpecimenCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (primaryTimer.time() > .5) {
                    hookSpecimenCase = hookSpecimenCases.DOWN;
                }
                break;
            case DOWN:
                primaryArm.setRotation(PrimaryArm.rotationStates.DOWN, .3, false);
                primaryShortcutCase = primaryShortcutCases.NONE;
                hookSpecimenCase = hookSpecimenCases.HOOK;
                break;
        }
    }

    public void retractFromChamber() {
        switch (retractFromChamberCase) {
            case OPEN:
                grabber.release();
                primaryTimer.reset();
                retractFromChamberCase = retractFromChamberCases.TIMEOUT1;
                break;
            case TIMEOUT1:
                if (primaryTimer.time() > .5) {
                    retractFromChamberCase = retractFromChamberCases.UP;
                }
                break;
            case UP:
                primaryArm.setRotation(PrimaryArm.rotationStates.UP, .5, false);
                primaryTimer.reset();
                retractFromChamberCase = retractFromChamberCases.TIMEOUT2;
                break;
            case TIMEOUT2:
                if (primaryTimer.time() > .5) {
                    retractFromChamberCase = retractFromChamberCases.RETRACT;
                }
                break;
            case RETRACT:
                primaryArm.setRetract();
                retractFromChamberCase = retractFromChamberCases.TUCK;
                break;
            case TUCK:
                grabber.headStraight();
                grabber.setGrabberState(Grabber.grabberStates.OUT);
                primaryShortcutCase = primaryShortcutCases.NONE;
                retractFromChamberCase = retractFromChamberCases.OPEN;
                break;
        }
    }


    private boolean secondaryDriverTwo = false;

    public void shortcuts() {
        if (secondaryDriverTwo) secondaryShortcuts();
        else primaryShortcuts();
    }

    private boolean primaryClipped = false;
    private boolean primaryDPressed = false;

    public void primaryShortcuts() {
        //Press gp2.a when grabber is aligned with specimen on wall.
        //This will grab specimen, lift arm slightly, tuck grabber, and retract arm.
        if (gamepad2.dpad_up) {
            primaryShortcutCase = primaryShortcutCases.GRAB_SPECIMEN;
        }

        //Press gp2.b when specimen is aligned with chamber, or after specimen has been hooked on chamber.
        else if (gamepad2.dpad_down) {
            //This will open the grabber, slightly raise the arm, retract the arm, and tuck grabber.
            if (primaryClipped && !primaryDPressed) {
                if (hookShortcutTimer.time(TimeUnit.SECONDS) > 5) {
                    primaryDPressed = true;
                    primaryShortcutCase = primaryShortcutCases.HOOK_SPECIMEN;
                    primaryClipped = true;
                    hookShortcutTimer.reset();
                    return;
                }
                primaryDPressed = true;
                primaryShortcutCase = primaryShortcutCases.RETRACT_FROM_CHAMBER;
                primaryClipped = false;
            }

            //This will set the grabber to the hook position and lower arm slightly.
            else if (!primaryDPressed) {
                primaryDPressed = true;
                primaryShortcutCase = primaryShortcutCases.HOOK_SPECIMEN;
                primaryClipped = true;
                hookShortcutTimer.reset();
            }
        } else if (gamepad2.dpad_left) {
            primaryClipped = false;
        } else {
            primaryDPressed = false;
        }
    }

    public void secondaryShortcuts() {
        //Press gp2.a when outgrabber is aligned with sample on floor.
        //This will grab sample, lift arm, and retract arm.
        if (gamepad2.dpad_up) {
            secondaryShortcutCase = secondaryShortcutCases.GRAB_SAMPLE;
        }
    }

    public void mechanismsControl() {
        if (secondaryDriverTwo) {
            outgrabberControl();
            secondaryArmControl();
        } else {
            grabberControl();
            primaryArmControl();
        }
    }

    public void grabberControl() {
        if (gamepad2.a) grabber.grab();
        else if (gamepad2.b) grabber.release();

        if (gamepad2.x) grabber.setGrabberState(Grabber.grabberStates.DOWN);
        else if (gamepad2.y) grabber.setGrabberState(Grabber.grabberStates.OUT);
        else if (gamepad2.dpad_right) grabber.doTuck();
        else if (gamepad2.right_stick_button) grabber.setGrabberState(Grabber.grabberStates.HOOK);

        else if (gamepad2.back) grabber.headStraight();
        else if (Math.abs(gamepad2.right_stick_x) > .35 || Math.abs(gamepad2.right_stick_y) > .35)
            grabber.rotate(gamepad2.right_stick_x, gamepad2.right_stick_y);
    }

    public void outgrabberControl() {
        if (gamepad2.b) outgrabber.release();
        else if (gamepad2.a) outgrabber.grab();

        if (gamepad2.y && gamepad2.left_bumper) outgrabber.midPosition();
        else if (gamepad2.y) outgrabber.upPosition();
        else if (gamepad2.x) outgrabber.downPosition();

        else if (gamepad2.back) outgrabber.headStraight();
        else if (Math.abs(gamepad2.right_stick_x) > .35 || Math.abs(gamepad2.right_stick_y) > .35)
            outgrabber.rotate(gamepad2.right_stick_x, gamepad2.right_stick_y);
    }

    public void primaryArmControl() {
        if (gamepad2.left_bumper && gamepad2.right_trigger > 0.35)
            primaryArm.extend(gamepad2.right_trigger);
        else if (gamepad2.right_trigger > 0.35) primaryArm.setExtend();
        else if (gamepad2.left_bumper && gamepad2.left_trigger > 0.35)
            primaryArm.retract(gamepad2.left_trigger);
        else if (gamepad2.left_trigger > 0.35) primaryArm.setRetract();

        if (gamepad2.start) {
            grabber.setGrabberState(Grabber.grabberStates.MANUAL);
        }

        if (gamepad2.left_stick_y < -.35 && !gamepad2.start) primaryArm.up(armSpeedMultiplier);
        else if (gamepad2.left_stick_y > .35 && !gamepad2.start)
            primaryArm.down(armSpeedMultiplier);
        else if (gamepad2.left_stick_y < -.35 && gamepad2.start) {
            grabber.tiltUp(Math.abs(gamepad2.left_stick_y));
            primaryArm.stopRotation();
        } else if (gamepad2.left_stick_y > -.35 && gamepad2.start) {
            grabber.tiltDown(Math.abs(gamepad2.left_stick_y));
            primaryArm.stopRotation();
        } else if (primaryArm.isStopped()) primaryArm.stopRotation();
    }

    public void secondaryArmControl() {
        if (gamepad2.left_bumper && gamepad2.right_trigger > 0.35) {
            secondaryArm.extend(gamepad2.right_trigger);
        } else if (gamepad2.right_trigger > 0.35) {
            if (secondaryArm.extender.getPosition() < .3) {
                outgrabber.upPosition();
            }
            secondaryArm.setExtend();
        } else if (gamepad2.left_bumper && gamepad2.left_trigger > 0.35) {
            secondaryArm.retract(gamepad2.left_trigger);
        } else if (gamepad2.left_trigger > 0.35) {
            if (secondaryArm.extender.getPosition() > .3) {
                outgrabber.headStraight();
                outgrabber.upPosition();
            }
            secondaryArm.setRetract();
        }
    }

    public void indicatorCheck() {
        double elapsedTime = totalTimer.time(TimeUnit.SECONDS);
        double remainingTime = 120 - elapsedTime;

        if (remainingTime > 75) {
            //First 50 seconds of TeleOp, gather samples from submersible
            if (grabber.isImuRunning()) {
                indicator.setColor(RGBIndicator.LightOptions.GREEN);
            } else {
                if ((int)remainingTime % 2 == 0) {
                    indicator.setColor(RGBIndicator.LightOptions.GREEN);
                } else {
                    indicator.setColor(RGBIndicator.LightOptions.RED);
                }
            }
        } else if (remainingTime > 70) {
            if ((int)remainingTime % 2 == 0) {
                indicator.setColor(RGBIndicator.LightOptions.GREEN);
            } else {
                indicator.setColor(RGBIndicator.LightOptions.OFF);
            }
        } else if (remainingTime > 35) {
            //First 40 seconds of hook specimen time
            if (grabber.isImuRunning()) {
                indicator.setColor(RGBIndicator.LightOptions.YELLOW);
            } else {
                if ((int)remainingTime % 2 == 0) {
                    indicator.setColor(RGBIndicator.LightOptions.YELLOW);
                } else {
                    indicator.setColor(RGBIndicator.LightOptions.RED);
                }
            }
        } else if (remainingTime > 30) {
            if ((int)remainingTime % 2 == 0) {
                indicator.setColor(RGBIndicator.LightOptions.YELLOW);
            } else {
                indicator.setColor(RGBIndicator.LightOptions.OFF);
            }
        } else if (remainingTime > 15) {
            //Last 10 seconds of match
            if (grabber.isImuRunning()) {
                indicator.setColor(RGBIndicator.LightOptions.ORANGE);
            } else {
                if ((int)remainingTime % 2 == 0) {
                    indicator.setColor(RGBIndicator.LightOptions.ORANGE);
                } else {
                    indicator.setColor(RGBIndicator.LightOptions.RED);
                }
            }
        } else if (remainingTime > 10) {
            if ((int)remainingTime % 2 == 0) {
                indicator.setColor(RGBIndicator.LightOptions.ORANGE);
            } else {
                indicator.setColor(RGBIndicator.LightOptions.OFF);
            }
        } else {
            indicator.setColor(RGBIndicator.LightOptions.RED);
        }
    }

    public void telemetryOutput() {
        telemetry.addData("Front Left: ", Bot.frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right: ", Bot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Rear Left: ", Bot.rearLeftMotor.getCurrentPosition());
        telemetry.addData("Rear Right: ", Bot.rearRightMotor.getCurrentPosition());

        telemetry.update();
    }

    public void stop() {
        indicator.setColor(RGBIndicator.LightOptions.OFF);
    }

}
