/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="HSV Test", group="Wobble Goal")
public class WobbleGoal_RedRight_HSV_Test extends LinearOpMode {
    double RIGHT_SLOW = -0.4;
    double TIMEOUT_WG_WALL = 1;
    double TIMEOUT_WG_TGC = 6.0;
    double FORWARD_SLOW= 0.3;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 8;
    int redColorFound = 0;
    float hue =0, saturation=0, value=0;

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    @Override
    public void runOpMode() {

        initHardwareMap();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()) {
            // Step 1: strafe to the wall
           // strafeRightToWall();

            //Step 2: move to Target Zone "C"
            //moveToTargetC();

            while ((elapsedTime.seconds()<30)  ){
                boolean isRedFound = isRedColorFound();


            }
        }

    }

    /**
     * Initialize Hardware and Map them.
     */
    private void initHardwareMap() {
        hardwarePushBot.mapWheels(hardwareMap);
        hardwarePushBot.mapColorSensor(hardwareMap);

        hardwarePushBot.leftColorSensor.enableLed(true);
        hardwarePushBot.rightColorSensor.enableLed(true);

        hardwarePushBot.setWheelDirection();
    }

    /**
     *  Strafe Right to the Wall.
     */
    // add touch sensor later, to improve the performance.
   private void strafeRightToWall() {
       while (elapsedTime.seconds()<TIMEOUT_WG_WALL) {
           hardwarePushBot.mecanumDrive(0, RIGHT_SLOW, 0);
       }
       hardwarePushBot.mecanumDrive(0,0,0);
   }

    /**
     * Robot landing in target C.
     */
    private void moveToTargetC() {
        elapsedTime.reset();
        redColorFound = 0;
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < 3 ){
            boolean isRedFound = isRedColorFound();
            hardwarePushBot.mecanumDrive(FORWARD_SLOW,0,0);
            if(isRedFound)
                redColorFound++;

        }

        hardwarePushBot.mecanumDrive(0,0,0);
        telemetry.addData("Color Red", hue);
        telemetry.addData("RedColorFound", redColorFound);
        telemetry.update();
        sleep(10000);
    }

    /**
     * If Color Sensor detects Red Color.
     * @return
     */
   public boolean isRedColorFound() {
        boolean found = false;

       Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
               (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
               (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
               hsvValues);
       // CheckForRed is a hsv value check

       hue = hsvValues[0];
       saturation = hsvValues[1];
       value = hsvValues[2];
       double lightIntensity;
       lightIntensity = hardwarePushBot.rightColorSensor.alpha();

       telemetry.addData("Hue", hue);
       telemetry.addData("Sat",  saturation);
       telemetry.addData("Val", value);
       telemetry.addData("LightIntensity",lightIntensity);
       telemetry.update();

       if((hue < 60 || hue > 320) ){
           found = true;
           sleep(50);
       }

       return found;
   }
}
