package org.firstinspires.ftc.teamcode.Anish;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Wobble_Goal_Blue_Right_Target_Zone_C", group = "TargetZoneC")

public class Anish_WobbleGoal_BlueRight_TargetZoneC extends LinearOpMode {


    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public RevColorSensorV3 colorSensor = null;

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    float hsvValues[] = {0, 0, 0};

    @Override
    public void runOpMode() throws InterruptedException {

        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        hardwarePushBot.setWheelDirection();


        waitForStart();

        while (opModeIsActive()) {

            leftFront.setPower(0.5);
            rightFront.setPower(-0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(-0.5);

            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            float value = hsvValues[2];

            if ((saturation < 0) && (value > 100)) {
                leftFront.setPower(-0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(0.5);
            }
            if (hue > 180 && hue < 240) {
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(-0.5);
            }
            if (hue > 180 && hue < 240) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
            }
        }
    }
}