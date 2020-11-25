package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "Red Right-Target A")
public class WobbleGoal_RedRight_TargetA extends LinearOpMode {

    double Right_Slow = -0.3;
    double Wait_Wall = 1;
    double Forward = 0.7;
    int ScaleFactor = 10;
    int Time = 4;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    int redColorFound = 0;
    float hue =0, saturation=0, value=0;

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    @Override
    public void runOpMode()  {
        initHardwaremap();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive());{
            //Robot strafes right to the wall
            strafeRightToWall();

            //Robot Moves to Target Zone A
            MoveToTargetZoneA();
        }

    }

    private void initHardwaremap() {
        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        hardwarePushBot.setWheelDirection();

    }

    private void strafeRightToWall() {
        while(elapsedTime.seconds()<Wait_Wall );{
        }

        hardwarePushBot.mecanumDrive(0, Right_Slow, 0);

        hardwarePushBot.mecanumDrive(0, 0, 0);

    }

    private void MoveToTargetZoneA() {
        elapsedTime.reset();
        redColorFound = 0;

        while ((elapsedTime.seconds()<Time) && redColorFound < 1 ){
            boolean isRedFound = isRedColorFound();

            if(isRedFound)
                redColorFound++;

        }

        hardwarePushBot.mecanumDrive(0,0,0);
        telemetry.addData("Color Red", hue);
        telemetry.addData("RedColorFound", redColorFound);
        telemetry.update();
        sleep(10000);

    }

    private boolean isRedColorFound() {
        boolean found = false;

        Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * ScaleFactor),
                (int) (hardwarePushBot.rightColorSensor.green() * ScaleFactor),
                (int) (hardwarePushBot.rightColorSensor.blue() * ScaleFactor),
                hsvValues);

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color Red", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.update();

        if((hue < 60 || hue > 320) ){
            found = true;
            sleep(50);
        }

        return found;


    }


}


