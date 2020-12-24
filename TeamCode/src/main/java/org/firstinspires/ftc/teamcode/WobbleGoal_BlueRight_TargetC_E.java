package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Eashan_WobbleGoal_BlueRight_targetC")
public class WobbleGoal_BlueRight_TargetC_E  extends LinearOpMode {
    double LEFT_SLOW = 0.4;
    double DRIVE_FORWARD = 0.3;
    double TIMEOUT_WG_WALL = 2.5;
    double TIMEOUT_WG_TGC = 6.0;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 8;
    float hue = 0, saturation = 0, value = 0;
    int blueColorFound = 0;

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();

        waitForStart();

        elapsedTime.reset();

        // Step 1: strafe left to the wall
        while (opModeIsActive()) {
            while (elapsedTime.seconds()<TIMEOUT_WG_WALL) {
                hardwarePushBot.mecanumDrive(0, LEFT_SLOW, 0);
            }
            hardwarePushBot.mecanumDrive(0,0,0);
        }
        //Step 2: move forward to Target Zone "C"
        elapsedTime.reset();
        blueColorFound = 0;
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && blueColorFound < 3 ){
            boolean isBlueFound = isBlueColorFound();
            hardwarePushBot.mecanumDrive(DRIVE_FORWARD,0,0);
            if(isBlueFound)
                blueColorFound++;

        }

        hardwarePushBot.mecanumDrive(0,0,0);
        sleep(10000);

    }

    private void initHardwareMap() {
        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        hardwarePushBot.setWheelDirection();
    }
    public boolean isBlueColorFound() {
        boolean found = false;

        Color.RGBToHSV(
                (int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR), hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        if((hue > 220 && hue < 260) ){
            found = true;
            sleep(50);
        }

        return found;
    }

}
