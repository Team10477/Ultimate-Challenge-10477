package org.firstinspires.ftc.teamcode.noveleven;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Nirvan")
public class NirvanPgm extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        leftBack = hardwareMap.get(DcMotor.class, "left_rear");
        rightBack = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        float hsvValuesLeft[] = {0F, 0F, 0F};
        final float valuesLeft[] = hsvValuesLeft;

        Color.RGBToHSV((int) (leftColorSensor.red() * 8), (int) (leftColorSensor.green() * 8), (int) (leftColorSensor.blue() * 8), hsvValuesLeft);

        float hue = hsvValuesLeft[0];

        boolean redHue = hue < 60 || hue > 320;

        boolean blueHue = hue > 120 || hue < 260;

        waitForStart();
        while (opModeIsActive()) {
            while (!blueHue) {
                leftFront.setPower(0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(0.5);
            }
            //Stop Robot for 1 second
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            sleep(1000);

            //Strafe to the left for 0.5 seconds
            leftFront.setPower(-0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(-0.5);

            sleep(500);

            //Stop for the robot for 1 second
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            sleep(1000);

            //Move forward for 0.5 seconds
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);

            sleep(500);

            //Stop the robot
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            sleep(1000);
        }
    }

}
