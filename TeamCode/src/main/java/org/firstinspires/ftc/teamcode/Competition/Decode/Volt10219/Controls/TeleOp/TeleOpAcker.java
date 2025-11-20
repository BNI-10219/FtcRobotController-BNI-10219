package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

import java.util.List;

//@Disabled
@TeleOp (name = "Acker TeleOp")
public class TeleOpAcker extends OpMode {
    double leftStickYVal;
    double leftStickXVal;
    double rightStickYVal;
    double rightStickXVal;

    double powerThreshold = 0;
    double speedMultiply = 1;

    private static final int AUDREY = 1;
    private static final int ANDREA = 2;
    private int currentProfile = ANDREA;

    double flSpeed;
    double frSpeed;
    double rlSpeed;
    double rrSpeed;

    public DecodeBot Bot = new DecodeBot();

    public IntakeState intakeState = IntakeState.READY;
    public OuttakeState outtakeState = OuttakeState.READY;

    private ElapsedTime intakeTimer = new ElapsedTime();

    private Timer outtakeTimer = new Timer();

    private Limelight3A limelight;
    public LLResult result;


    @Override
    public void init() {
        Bot.initRobot(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();


    }

    public void init_loop() {
    }


    public void start() {}

    @Override
    public void loop() {
        speedControl();
        launcherControl();
        //changeDriverProfile();
        intakeControl();
        artifactPushControl();
        telemetryOutput();
        fieldCentricDrive();
        autoPositioningV2();
        intakeControlStates();
        timeOuttake();
    }

    public void changeDriverProfile() {
        if (gamepad1.left_trigger > 0.01) {
            currentProfile = AUDREY;
        }
        else if (gamepad1.left_bumper) {
            currentProfile = ANDREA;
        }

    }

    public void autoPositioningV2() {

        // Only run auto-align when holding left bumper
        if (!gamepad1.left_bumper) {
            return;
        }

        // Get latest Limelight result
        result = limelight.getLatestResult();

        // Edge Case Handling in Case No Limelight Results
        if (result == null) {
            telemetry.addData("LL", "No result (null)");
            return;
        }
        // Edge Case Handling in Case No Fiducial Results
        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            telemetry.addData("LL", "No fiducial targets");
            return;
        }

        // --- Choose the best tag (ID 20 or 24) closest to center ---
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) {
                if (best == null ||
                        Math.abs(fr.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) {
                    best = fr;
                }
            }
        }

        // Edge Case Handling in for some reason you detect Motif APril Tags
        if (best == null) {
            telemetry.addData("LL", "No desired tag (20/24) in view");
            return;
        }

        // Get target degrees of April Tag if no edge cases
        double tx = best.getTargetXDegrees();

        // --- Proportional Drive Control parameters  ---
        double kP = 0.03;             // Proportional gain for turning and oscillation
        double maxTurnSpeed = 0.50;  // Max turn power
        double minTurnSpeed = 0.25;  // Minimum turn power to overcome friction
        double tolerance = 1.5;      // Deadband in degrees that controls oscillation

        // If we’re close enough, stop and don’t oscillate
        if (Math.abs(tx) < tolerance) {
            Bot.stopMotors();
            telemetry.addData("Align", "Aligned! tx=%.2f", tx);
            telemetry.addData("Tag", "ID: %d", best.getFiducialId());
            return;
        }

        // Proportional turning power
        double power = kP * tx;

        // Clip to max speed
        if (power > maxTurnSpeed) power = maxTurnSpeed;
        if (power < -maxTurnSpeed) power = -maxTurnSpeed;

        // Enforce a minimum power when we’re still outside tolerance
        if (power > 0 && Math.abs(power) < minTurnSpeed) power = minTurnSpeed;
        if (power < 0 && Math.abs(power) < minTurnSpeed) power = -minTurnSpeed;

        // Map sign so that:
        //  tx < 0 (tag left) so robot turns left (fl -, fr +)
        //  tx > 0 (tag right) so robots turns right (fl +, fr -)
        double fl =  power;
        double fr = -power;
        double rl =  power;
        double rr = -power;

        // Set motor powers
        setMotorPower(Bot.flMotor, fl, powerThreshold, 1.0);
        setMotorPower(Bot.frMotor, fr, powerThreshold, 1.0);
        setMotorPower(Bot.rlMotor, rl, powerThreshold, 1.0);
        setMotorPower(Bot.rrMotor, rr, powerThreshold, 1.0);

        telemetry.addData("Align", "tx=%.2f, power=%.2f", tx, power);
        telemetry.addData("Tag", "ID: %d", best.getFiducialId());
    }

    // ****** Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }

    public void fieldCentricDrive(){
        switch(currentProfile){
            case ANDREA:
                double y;
                double x;
                double rx;

                y = Math.abs(Math.pow(gamepad1.left_stick_y, 2));// Remember, Y stick value is reversed
                if (gamepad1.left_stick_y > 0) y *= -1;
                x = Math.abs(Math.pow(gamepad1.left_stick_x, 2));
                if (gamepad1.left_stick_x < 0) x *= -1;
                rx = Math.abs(Math.pow(gamepad1.right_stick_x, 2));
                if (gamepad1.right_stick_x < 0) rx *= -1;


                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if(gamepad1.options){
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
                double flPower = (rotY + rotX + rx) / denominator;
                double rlPower = (rotY - rotX + rx) / denominator;
                double frPower = (rotY - rotX - rx) / denominator;
                double rrPower = (rotY + rotX - rx) / denominator;

                Bot.flMotor.setPower(flPower);
                Bot.frMotor.setPower(frPower);
                Bot.rlMotor.setPower(rlPower);
                Bot.rrMotor.setPower(rrPower);
                break;
            case AUDREY:
                leftStickYVal = -gamepad1.left_stick_y;
                leftStickYVal = Range.clip(leftStickYVal, -1, 1);
                leftStickXVal = gamepad1.left_stick_x;
                leftStickXVal = Range.clip(leftStickXVal, -1, 1);
                rightStickXVal = gamepad1.right_stick_x;
                rightStickXVal = Range.clip(rightStickXVal, -1, 1);

                //Robot Centric - Andrea prefers
                flSpeed = leftStickYVal + rightStickXVal + leftStickXVal;    // Vertical + Rotation + Staffing
                flSpeed = Range.clip(flSpeed, -1, 1);
                frSpeed = leftStickYVal - rightStickXVal - leftStickXVal;   // Vertical - Rotation - Strafing(sign in front is the way the motor is turning in relation to the others)
                frSpeed = Range.clip(frSpeed, -1, 1);
                rlSpeed = leftStickYVal - rightStickXVal + leftStickXVal;
                rlSpeed = Range.clip(rlSpeed, -1, 1);
                rrSpeed = leftStickYVal + rightStickXVal - leftStickXVal;
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
                if(gamepad1.right_bumper){
                    Bot.resetHeading();
                }
                break;
        }
        if (gamepad1.right_trigger > 0.1){
            Bot.flMotor.setPower(.5);
            Bot.frMotor.setPower(.5);
            Bot.rlMotor.setPower(.5);
            Bot.rrMotor.setPower(.5);
        }
    }


    public void launcherControl() {
//        if(gamepad2.x){
//            Bot.ballLaunchV();
//        }

        if (gamepad2.y) {
            Bot.ballLaunchStop();
        }
        if (gamepad2.right_bumper) {
            Bot.ballLaunchMidV();
        }
        if (gamepad2.right_trigger > 0.001) {
            Bot.ballLaunchBackField();
        }
    }

    public enum IntakeState {
        RUN,
        WAIT,
        STOP,
        READY;
    }

    public void intakeControlStates() {
        switch (intakeState) {
            case RUN:
                Bot.ballIntake();
                intakeTimer.reset();
                intakeState = IntakeState.WAIT;
                break;
            case WAIT:
                if (intakeTimer.time() > 1.5) {
                    intakeState = IntakeState.STOP;
                }
                break;
            case STOP:
                Bot.intakeStop();
                intakeState = IntakeState.READY;
                break;
            case READY:
                break;
        }
    }

    public enum OuttakeState {START, WAIT, STOP, READY}

    public void timeOuttake() {
        switch (outtakeState) {
            case START:
                Bot.ballOuttake();
                outtakeState = OuttakeState.WAIT;
                outtakeTimer.resetTimer();
                break;
            case WAIT:
                if (outtakeTimer.getElapsedTimeSeconds() > 0.005) {
                    outtakeState = OuttakeState.STOP;
                }
                break;
            case STOP:
                Bot.intakeStop();
                outtakeState = OuttakeState.READY;
                break;
            case READY:
                break;
        }
    }

    public void intakeControl() {
        if (gamepad2.dpad_right) {
            Bot.ballIntake();
        }
        if (gamepad2.dpad_up)
            intakeState = IntakeState.RUN;

        if (gamepad2.dpad_left) {
            Bot.ballOuttake();
        }
        if (gamepad2.b) {
            Bot.intakeStop();
        }
        if (gamepad2.x) {
            outtakeState = OuttakeState.START;
        }
    }

    public void artifactPushControl() {
        if (gamepad1.y) {
            Bot.artifactPushUps();
        }

        if (gamepad1.a) {
            Bot.artifactPushDown();
        }
        if (gamepad1.b) {
            Bot.artifactPushMiddle();
        }
    }

        public void speedControl () {
            if (gamepad1.left_bumper) {
                speedMultiply = 1;
            } else if (gamepad1.left_trigger > 0.1) {
                speedMultiply = 0.5;
            }
        }

        public void telemetryOutput () {

            telemetry.addData("Launcher One: ", Math.abs(Bot.ballLaunchOne.getVelocity()));
            telemetry.addData("Launcher Two: ", Bot.ballLaunchTwo.getVelocity());

            //telemetry.addData("Limelight AT: ", result.getFiducialResults());
            telemetry.update();
        }
    }
