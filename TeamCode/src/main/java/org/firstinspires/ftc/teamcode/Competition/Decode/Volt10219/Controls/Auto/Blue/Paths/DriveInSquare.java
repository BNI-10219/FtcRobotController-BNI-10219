package org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.Paths;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Competition.Decode.Volt10219.Controls.Auto.Blue.BlueAlliance;

@Autonomous(name = "Drive In Square")
public class DriveInSquare extends BlueAlliance {

    public void runOpMode() throws InterruptedException{
        Bot.initRobot(hardwareMap);

        while (opModeIsActive()) {

            //drive code
            while (opModeIsActive()) {

                Bot.driveForward(.25, 1);
                sleep(1000);
                Bot.strafeLeft(.25, 1);
                sleep(1000);
                Bot.driveBack(.25, 1);
                sleep(1000);
                Bot.strafeRight(.25, 1);
                sleep(1000);

                requestOpModeStop();

            }
            idle();
        }

    }
}
