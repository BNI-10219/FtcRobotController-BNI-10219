package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Blue17241.Robots.ProgrammerBot;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp (name = "RGB Light Color Sensor Tester")
public class TesterRGBLightSensor extends OpMode{

    //Instantiate Desired Robot
    public ProgrammerBot Bot = new ProgrammerBot();

    //Instantiate RGB Light and Sensor
    public RGBLight led = new RGBLight();
    public ColorDistSensor sensor = new ColorDistSensor();


    //Initiliaze Variables for Time Tracking
    public ElapsedTime timer = new ElapsedTime();
    public double remainingTime;
    public double elapsedTime;

    //Initiliaze and Declare Variables for Color Enum
    public RGBLight.ColorOptions color = RGBLight.ColorOptions.OFF;

    //Initiliaze and Declare Color Variables
    public double lowThresh = 200;
    public double highThresh = 300;
    public double redScaled = 0;
    public double greenScaled = 0;
    public double blueScaled = 0;
    public double hueScaled = 0;


    @Override
    public void init() {

        Bot.initRobot(hardwareMap);
        led.initRGBLight(hardwareMap);
        sensor.initColorDistSensor(hardwareMap);
    }

    @Override
    public void start() {
        led.setColor(RGBLight.ColorOptions.OFF);
        timer.reset();
    }

    @Override
    public void loop() {
        lightTimerControl();
        lightColorSensorControl();
        //lightDistanceSensorControl();
        telemetry();
        //telemetry_dist();

    }


    public void lightTimerControl() {
        elapsedTime = timer.time(TimeUnit.SECONDS);
        remainingTime = 120 - elapsedTime;

        if (remainingTime < 10) {
            led.setColor(RGBLight.ColorOptions.PURPLE);
            color = RGBLight.ColorOptions.PURPLE;
        }
        else if (remainingTime < 20) {
            led.setColor(RGBLight.ColorOptions.ORANGE);
            color = RGBLight.ColorOptions.ORANGE;
        }

    }

    public void lightColorSensorControl() {

        sensor.convertColors();

        // SCALING 0 to 1 values to 0 to 255 values
        redScaled = sensor.colorSensor.red();
        greenScaled = sensor.colorSensor.green();
        blueScaled = sensor.colorSensor.blue();
        hueScaled = sensor.hsvValues[0];

        // YELLOW SAMPLE  (Red:255 ; Green:255 ; Blue: 0 ; Hue: 30-90)
        //BLUE SAMPLE  (Red:0 ; Green:0 ; Blue: 255 ; Hue 200)
        //RED SAMPLE   (Red:255 ; Green:0 ; Blue: 0; Hue: 10)
        if (hueScaled < 30) {
            led.setColor(RGBLight.ColorOptions.RED);
        }

        else if (hueScaled > 200) {
            led.setColor(RGBLight.ColorOptions.BLUE);
        }

        else if (hueScaled > 30 && hueScaled < 90) {
            led.setColor(RGBLight.ColorOptions.YELLOW);
        }
        else {
            led.setColor(RGBLight.ColorOptions.OFF);
        }
    }


    public void lightDistanceSensorControl() {

        if (sensor.distanceSensor.getDistance(DistanceUnit.INCH) < 2.0) {
            led.setColor(RGBLight.ColorOptions.PURPLE);
        }

    }


    public void telemetry(){

        telemetry.addData("RGB Light Color: ", color);
        telemetry.addData("Elapsed Time: ", elapsedTime);
        telemetry.addData("Remaining Time: ", remainingTime);
        telemetry.addData("Red Value: ", sensor.colorSensor.red());
        telemetry.addData("Green Value: ", sensor.colorSensor.green());
        telemetry.addData("Blue Value: ", sensor.colorSensor.blue());
        telemetry.addData("Hue Value: ", sensor.hsvValues[0]);
        telemetry.update();
    }

    public void telemetry_dist(){
        telemetry.addData("Distance (inches): ", String.format(Locale.US, "%.02f", sensor.distanceSensor.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }


}
