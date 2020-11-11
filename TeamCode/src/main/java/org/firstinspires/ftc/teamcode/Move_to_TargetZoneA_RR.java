package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import static android.os.SystemClock.sleep;

@Autonomous
public class Move_to_TargetZoneA_RR extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;


    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "left rear");
        rightBack = hardwareMap.get(DcMotor.class, "right rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor_front");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);


        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);


    }

    @Override
    public void loop() {

        sleep(1000);
        leftFront.setPower(0.25);
        rightFront.setPower(-0.25);
        rightBack.setPower(-0.25);
        leftBack.setPower(0.25);

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        float hue = hsvValues[0];

        Color.RGBToHSV((int) (leftColorSensor.red() * 8), (int) (leftColorSensor.green() * 8), (int) (leftColorSensor.blue() * 8), hsvValues);


        telemetry.addData("Color Red", hue);
        telemetry.update();

        boolean redHue = hue < 60 || hue > 320;

        if (redHue) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        } else {
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);


        }
    }

}

