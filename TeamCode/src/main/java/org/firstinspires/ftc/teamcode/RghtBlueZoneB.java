package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class RghtBlueZoneB extends LinearOpMode {
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public RevColorSensorV3 ColorSensor = null;
    public RevColorSensorV3 RightColorSensor = null;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class,"left_front");
        rightFront = hardwareMap.get(DcMotor.class,"right_front");
        leftBack = hardwareMap.get(DcMotor.class,"left_rear");
        rightBack = hardwareMap.get(DcMotor.class,"right_rear");
        ColorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor");
        RightColorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        ColorSensor.enableLed(true);
        RightColorSensor.enableLed(true);
        boolean colorFound = false;

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        int counter = 0;

        waitForStart();
        Color.RGBToHSV((int)(ColorSensor.red() * 8), (int)(ColorSensor.green() *8), (int)(ColorSensor.blue() * 8), hsvValues);
        Color.RGBToHSV((int)(RightColorSensor.red() * 8), (int)(RightColorSensor.green() *8), (int)(RightColorSensor.blue() * 8), hsvValues);
        float hue = hsvValues[0];
        boolean redHue = hue < 60 || hue > 320;
        boolean blueHue = hue > 120 && hue < 260;

        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightBack.setPower(0.5);

        if(blueHue = true) {
            counter++;
            if (counter == 2) {
                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(0.5);

                sleep(500);

                leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(-0.5);

                if (blueHue == true) {
                    leftFront.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    rightBack.setPower(0);
                }
            }
        }
    }
}

