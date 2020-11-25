package org.firstinspires.ftc.teamcode.noveleven;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "WobbleGoal_TargetZoneC_BlueRight", group = "TargetZoneC")
public class WobbleGoal_TargetZoneC_BlueRight extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public RevColorSensorV3 colorSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_rear");
        rightBack = hardwareMap.get(DcMotor.class, "right_rear");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        colorSensor.enableLed(true);

        float hsvValues[] = {0, 0, 0};

        waitForStart();

        while (opModeIsActive()) {
            leftFront.setPower(0.5);
            rightFront.setPower(-0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(-0.5);

            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            float value = hsvValues[2];

            if ((hue > 0 && hue < 360) && (saturation < 0) && (value > 100)) {
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
