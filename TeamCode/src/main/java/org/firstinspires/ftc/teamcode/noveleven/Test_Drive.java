package org.firstinspires.ftc.teamcode.noveleven;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static android.os.SystemClock.sleep;

@Autonomous(name="TEST1")
public class Test_Drive extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;

    @Override
    public void init() {

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

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);
    }

    @Override
    public void loop() {

        leftFront.setPower(0.25);
        leftBack.setPower(0.25);
        rightFront.setPower(0.25);
        rightBack.setPower(0.25);

        sleep(1000)    ;
        leftFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
        rightBack.setPower(0.0);

    }

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        leftBack = hardwareMap.get(DcMotor.class, "left_rear");
        rightBack = hardwareMap.get(DcMotor.class, "right_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
    }
}
