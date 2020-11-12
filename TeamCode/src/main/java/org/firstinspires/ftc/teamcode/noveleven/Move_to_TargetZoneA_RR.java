package org.firstinspires.ftc.teamcode.noveleven;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Red_Right_Target_Zone")
public class Move_to_TargetZoneA_RR extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    double  leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower =0;
    double rightRearPower = 0;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;


    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotor.class, "left rear");
        rightBack = hardwareMap.get(DcMotor.class, "right rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor_right");

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

        mecanumDrive(0, -0.4, 0);

        Color.RGBToHSV((int) (leftColorSensor.red() * 8), (int) (leftColorSensor.green() * 8), (int) (leftColorSensor.blue() * 8), hsvValues);

        float hue = hsvValues[0];

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

    public void mecanumDrive(double drive, double strafe, double turn){
        leftFrontPower   = Range.clip(drive+turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(drive-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(drive+turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(drive-turn-strafe , -1.0, 1.0);

        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftRearPower);
        rightBack.setPower(rightRearPower);
    }
}

