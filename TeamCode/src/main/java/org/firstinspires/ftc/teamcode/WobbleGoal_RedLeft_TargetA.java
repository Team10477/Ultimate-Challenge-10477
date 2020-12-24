package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedLeft_TargetA_Ajay")
public class WobbleGoal_RedLeft_TargetA extends LinearOpMode {

    public HardwarePushBot robot = new HardwarePushBot();

    float hsvValues[] = {0F, 0F, 0F};
    float HsvRed[] = {0f, 0f, 100f};
    float HsvWhite[] = {0f, 0f, 100f};
    double SCALE_FACTOR = 8.0;
    float hue = 0, saturation = 0, value = 0;
    private ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
      robot.mapWheels(hardwareMap);
      robot.mapColorSensor(hardwareMap);
      robot.leftColorSensor.enableLed(true);
      robot.rightColorSensor.enableLed(true);
      robot.setWheelDirection();

      waitForStart();
      elapsedTime.reset();
      while (opModeIsActive()) {
          //1. Strafe Left for few msecs
           strafeLeft();

          //2. Stop at white

          //3. Strafe right until color sensor senses Red or touch sensor touches wall.
      }
    }

    /**
     *  Strafe Left for 5 msec.
     */
    // add touch sensor later, to improve the performance.
    private void strafeLeft() {
        while (elapsedTime.seconds() < 0.5) {
            robot.mecanumDrive(0, 0.34, 0);
        }
        robot.mecanumDrive(0,0,0);
    }

    private void stopAtWhite() {
        boolean whiteColorFound = false;
       while (!whiteColorFound ){
           whiteColorFound = isWhiteColorFound();
           robot.mecanumDrive(0.35, 0, 0);

        }

        robot.mecanumDrive(0,0,0);
        telemetry.addData("Color White", hue);
        telemetry.addData("RedColorFound", whiteColorFound);
        telemetry.update();
        sleep(10000);

    }

    /**
     * If Color Sensor detects White Color.
     * @return
     */
    public boolean isWhiteColorFound() {
        boolean found = false;

        Color.RGBToHSV((int) (robot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (robot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (robot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);


        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color White", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.update();

        if(value >= 40){
            found = true;
            sleep(50);
        }

        return found;
    }

    /**
     * If Color Sensor detects Red Color.
     * @return
     */
    public boolean isRedColorFound() {
        boolean found = false;

        Color.RGBToHSV((int) (robot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (robot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (robot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);


        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color White", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.update();

        if((hue < 60 || hue > 320) ){
            found = true;
            sleep(50);
        }

        return found;
    }
}
