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


@Autonomous(name="Red Right-Target B", group="Wobble Goal")
public class WobbleGoal_RedRight_TargetB extends LinearOpMode {
    double RIGHT_SLOW = -0.4;
    double TIMEOUT_WG_WALL = 1;
    double TIMEOUT_WG_TGC = 6.0;
    double STRAFE_TIME_RED_RIGHT = 0.5;
    double STRAFE_TIME_RED_LEFT = 0.5;
    double LEFT_SLOW = 0.4;

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
            strafeRightToWall();

            //Step 2: detect two red
            moveToTargetC(2);

            // Step 3: Go forward to align with Target B
            reachTargetB();
            //Step 4: strafe to red line
            strafe_to_red_left(1);
            //Step 5: Deposit WB and return

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
    private void reachTargetB() {
        while (elapsedTime.seconds() < STRAFE_TIME_RED_RIGHT) {
            hardwarePushBot.mecanumDrive(RIGHT_SLOW, 0, 0);
        }
        hardwarePushBot.mecanumDrive(0, 0, 0);
    }
    /**
     * Robot landing in target C.
     */
    private void moveToTargetC(int redcolornum) {
        elapsedTime.reset();
        redColorFound = 0;
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < redcolornum ){
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
    // strafe left to red
    private void strafe_to_red_left(int numberOfRed) {
        while (elapsedTime.seconds() < STRAFE_TIME_RED_LEFT&& redColorFound < numberOfRed ) {
            boolean isRedFound = isRedColorFound();

            if(isRedFound)
                redColorFound++;
            hardwarePushBot.mecanumDrive(0, LEFT_SLOW, 0);
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

       telemetry.addData("Color Red", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
       telemetry.update();

       if((hue < 60 || hue > 320) ){
           found = true;
           sleep(50);
       }

       return found;
   }
}
