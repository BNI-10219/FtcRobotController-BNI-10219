package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Robot.DecodeBot;

@TeleOp(name = "Velocity Gate Tester", group = "Tester")
public class VelocityGateTester extends OpMode {

    // Flywheel & Feed Wheel Variables
    public double targetVelocity = 0;

    // Velocity gate
    double gatePercent = 0.03;            // ±3% gate window

    // Feed action
    double feederPower = 1.0;             // power for feeder wheel (0..1)
    long   feedMs = 700;                  // how long to run feeder

    // Shot-drop compensation (temporary target bump while feeding)
    double boostFactor = 1.02;            // +2% target during feed
    long   boostMs = 180;                 // usually ~150–250 ms

    // ===== Feeder FlyWheel Gate State Control =====
    enum ShootState { IDLE, WAIT_FOR_GATE, FEEDING, RECOVERING }
    ShootState state = ShootState.IDLE;
    ElapsedTime timer = new ElapsedTime();

    boolean rb;
    boolean rbPressed;
    boolean prevRb;


    double nominalTarget = 0;             // remembers non-boosted target
    double tolerance; // floor to 10 ticks per secibd
    boolean leftInGateStatus = false;
    boolean rightInGateStatus = false;
    boolean inGate = false;

    double currentVelocityLeft;
    double currentVelocityRight;

    // Instantiation of Robot using Robot Class Constructor
    public DecodeBot Bot = new DecodeBot();


    @Override
    public void init() {
        Bot.initRobot(hardwareMap);
        Bot.imu.resetYaw();                   // REV
    }


    @Override
    public void loop() {
        telemetryOutput();
        flyWheelControl();
        flyWheelStateControl();
        //feedWheelManualControl();
    }

    //*********  Driver 2 Control Methods *****************


    // Fly Wheel Control
    public void flyWheelControl() {

        if (gamepad2.x) { targetVelocity = 876; }
        if (gamepad2.a) { targetVelocity = 934; }
        if (gamepad2.b) { targetVelocity = 1003; }
        if (gamepad2.y) { targetVelocity = 1125; }
        if (gamepad2.dpad_up) targetVelocity += 1;
        if (gamepad2.dpad_down) targetVelocity -= 1;
        if (gamepad2.left_bumper) { targetVelocity = 0; }

        Bot.ballLaunchOne.setVelocity(targetVelocity);
        Bot.ballLaunchTwo.setVelocity(targetVelocity);

        // Keep nominalTarget synced unless we’re in a boost
        if (state == ShootState.IDLE || state == ShootState.WAIT_FOR_GATE) {
            nominalTarget = targetVelocity;
        }

        // Always command velocity each loop
        Bot.ballLaunchOne.setVelocity(targetVelocity);
        Bot.ballLaunchTwo.setVelocity(targetVelocity);

        // ===== Read velocities & gate =====
        currentVelocityLeft = Bot.ballLaunchOne.getVelocity();
        currentVelocityRight = Bot.ballLaunchTwo.getVelocity();

        tolerance = Math.max(10.0, Math.abs(nominalTarget) * gatePercent); // floor to 10 ticks per secibd
        leftInGateStatus  = Math.abs(currentVelocityLeft - nominalTarget) <= tolerance;
        rightInGateStatus = Math.abs(currentVelocityRight - nominalTarget) <= tolerance;
        inGate = leftInGateStatus && rightInGateStatus;

        // ===== Shot request (rising edge on RB) =====
        rb = gamepad2.right_bumper;
        rbPressed = rb && !prevRb;
        prevRb = rb;

    }

    public void flyWheelStateControl() {
        // ===== State machine =====
        switch (state) {
            case IDLE:
                Bot.ballLaunchOne.setPower(0);
                Bot.ballLaunchTwo.setPower(0);
                if (rbPressed) {
                    state = ShootState.WAIT_FOR_GATE;
                }
                break;

            case WAIT_FOR_GATE:
                Bot.ballLaunchOne.setPower(0);
                Bot.ballLaunchTwo.setPower(0);
                if (inGate) {
                    // Apply brief boost and feed
                    targetVelocity = nominalTarget * boostFactor;
                    timer.reset();
                    Bot.ballLaunchOne.setPower(feederPower);
                    Bot.ballLaunchTwo.setPower(feederPower);
                    state = ShootState.FEEDING;
                }
                // If driver cancels by pressing X (stop), go idle
                if (gamepad2.yWasPressed()) {
                    state = ShootState.IDLE;
                    targetVelocity = nominalTarget;
                }
                break;

            case FEEDING:
                // Maintain boost while feeding
                targetVelocity = nominalTarget * boostFactor;
                if (timer.milliseconds() >= feedMs) {
                    Bot.ballLaunchOne.setPower(0);
                    Bot.ballLaunchTwo.setPower(0);
                    // Start recovery (let wheel return to nominal target)
                    targetVelocity = nominalTarget;
                    timer.reset();
                    state = ShootState.RECOVERING;
                }
                break;

            case RECOVERING:
                // Give the wheel a short window to re-settle; you can also re-arm immediately.
                if (timer.milliseconds() >= boostMs) {
                    state = ShootState.IDLE;
                }
                break;
        }



    }
    // ***** Manual Feeder Wheel Controller
//    public void feedWheelManualControl() {
//        if (gamepad2.left_trigger > 0.5) {
//            Bot.ballLaunchOne.setPower(1);
//            Bot.ballLaunchTwo.setPower(1);;
//        }
//        else if (gamepad2.right_trigger > 0.5) {
//            Bot.ballLaunchOne.setPower(-1);
//            Bot.ballLaunchTwo.setPower(-1);
//        }
//        else if(gamepad2.left_stick_button){
//            Bot.ballLaunchOne.setPower(0);
//            Bot.ballLaunchTwo.setPower(0);
//        }
//
//    }


    // ***** Helper Method for Telemetry
    public void telemetryOutput() {
        telemetry.addData("Target Velocity: ", targetVelocity);
        telemetry.addData("Left Fly Wheel Velocity: ", Bot.ballLaunchOne.getVelocity());
        telemetry.addData("Right Fly Wheel Velocity: ", Bot.ballLaunchTwo.getVelocity());
        telemetry.addData("Launching State", state);
        telemetry.addData("Target (nominal velocity)", nominalTarget);
        telemetry.addData("Target (cmd velocity)", targetVelocity);
        telemetry.addData("Gate Tolerance ±%", gatePercent * 100.0);
        telemetry.addData("Tolerance (ticks per sec)", tolerance);
        telemetry.addData("Left Fly Wheel velocity", currentVelocityLeft);
        telemetry.addData("Right Fly Wheel velocity", currentVelocityRight);
        telemetry.addData("Left|Right inGate Status", "%b | %b", leftInGateStatus, rightInGateStatus);
        telemetry.addData("Ball Launch One", Bot.ballLaunchOne.getPower());
        telemetry.addData("Ball Launch Two", Bot.ballLaunchTwo.getPower());
        telemetry.update();
        telemetry.update();
    }


    // ****** Helper method to set Motor Power
    public void setMotorPower(DcMotor motor, double speed, double threshold, double multiplier) {
        if (speed <= threshold && speed >= -threshold) {
            motor.setPower(0);
        } else {
            motor.setPower(speed * multiplier);
        }
    }



}