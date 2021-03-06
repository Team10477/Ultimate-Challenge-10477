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


@Autonomous(name="Red Left Target VG", group="Wobble Goal")
public class WobbleGoal_RedLeft_VG extends LinearOpMode {
    double RIGHT_SLOW = -0.4;
    double LEFT_SLOW = 0.4;
    double TIMEOUT_WG_WALL = 1;
    double STRAFE_TIME_RED_LEFT = 0.5;
    double STRAFE_TIME_RED_RIGHT = 0.5;
    double TIMEOUT_WG_TGC = 6.0;
    double LIGHT_INTENSITY_WHITE = 40;
    double FORWARD_SLOW = 0.25;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 8;
    int redColorFound = 0;
    int whitecolorfound = 0;
    float hue = 0, saturation = 0, value = 0;
    boolean isStop = false;
    int TargetZone = 0;
    HardwarePushBot hardwarePushBot = new HardwarePushBot();
    FeedbackMovement feedbackMovement = new FeedbackMovement();
    TensorFlowIdentifyTargetZone tensorFlowIdentifyTargetZone = new TensorFlowIdentifyTargetZone();


    @Override
    public void runOpMode() {

        initHardwareMap();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        elapsedTime.reset();


        while (opModeIsActive() && !isStop) {

            //Identify target zone
            tensorFlowIdentifyTargetZone.identifyRings();
            sleep(1000);
            TargetZone = tensorFlowIdentifyTargetZone.targetZone;
            tensorFlowIdentifyTargetZone.deactivateTfod();
            //Using TensorFlow to navidate to targetzone a nd then deposit WB
            switch(TargetZone) {
                case 1:
                    // Navigate to Target zone A
                    break;
                case 2:
                    // Navigate to Target zone B
                    break;
                case 3:
                    //Navigate to Target zone C
                    break;

                default:
                    // TFOD confused
            }


            // step 1: strafe left half a square
            strafeLeft();
            //Step 2: move straight until white line
            movetowhiteline();

            // Step 3: strafe to the right until red line
            strafe_to_redRight(1);
            // Step 4: strafe right to center in target zone.


            // Step 4: deposit WB and return
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

        feedbackMovement.initializeImu(hardwareMap);

        tensorFlowIdentifyTargetZone.initVuforia();
        tensorFlowIdentifyTargetZone.initTfod();
        tensorFlowIdentifyTargetZone.activateTfod();
    }

    /**
     * Strafe Right to the Wall.
     */
    // add touch sensor later, to improve the performance.
    private void strafeLeft() {
        while (elapsedTime.seconds() < STRAFE_TIME_RED_LEFT) {
            hardwarePushBot.mecanumDrive(0, LEFT_SLOW, 0);
           // feedbackMovement.initIntegralError(LEFT_SLOW, hardwarePushBot);
            //feedbackMovement.driveWithFeedback(hardwarePushBot,0,LEFT_SLOW);
        }
       //hardwarePushBot.mecanumDrive(0, 0, 0);
        feedbackMovement.driveWithFeedback(hardwarePushBot,0,0);
    }

    // Strafe right to red line
    private void strafe_to_redRight(int numberofred){
        elapsedTime.reset();
        redColorFound = 0;
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < numberofred ){
            boolean isRedFound = isRedColorFound();
            hardwarePushBot.mecanumDrive(0, RIGHT_SLOW, 0);


           // feedbackMovement.initIntegralError(RIGHT_SLOW, hardwarePushBot);
           // feedbackMovement.driveWithFeedback(hardwarePushBot,0,RIGHT_SLOW);
            if(isRedFound)
                redColorFound++;

        }

    //    feedbackMovement.initIntegralError(RIGHT_SLOW, hardwarePushBot);
        feedbackMovement.driveWithFeedback(hardwarePushBot,0,0);
       // hardwarePushBot.mecanumDrive(0,0,0);
        telemetry.addData("Color Red", hue);
        telemetry.addData("RedColorFound", redColorFound);
        telemetry.update();

    }


    /**
     * Robot landing in target C.
     */
    // Detect white line

    private void movetowhiteline() {
        elapsedTime.reset();
        redColorFound = 0;
        whitecolorfound = 0;
        boolean iswhitefound = false;
        while ((elapsedTime.seconds() < TIMEOUT_WG_TGC) && !iswhitefound) {
           iswhitefound = iswhitefound();
            if (!iswhitefound)
                 hardwarePushBot.mecanumDrive(FORWARD_SLOW, 0, 0);
                //feedbackMovement.initIntegralError(FORWARD_SLOW, hardwarePushBot);
           // feedbackMovement.driveWithFeedback(hardwarePushBot,FORWARD_SLOW,0);

           /* if (iswhitefound)
                whitecolorfound++;
*/
        }
        feedbackMovement.driveWithFeedback(hardwarePushBot,0,0);
     //  hardwarePushBot.mecanumDrive(0, 0, 0);
        isStop = true;
        telemetry.addData("Color white", hue);
        telemetry.addData("WhiteColorFound", redColorFound);
        telemetry.update();
    }

    // strafe right
    private void strafeRight() {
        elapsedTime.reset();
        while (elapsedTime.seconds() < STRAFE_TIME_RED_RIGHT) {
            hardwarePushBot.mecanumDrive(0, RIGHT_SLOW, 0);
            //feedbackMovement.driveWithFeedback(hardwarePushBot,0,RIGHT_SLOW);
        }
        hardwarePushBot.mecanumDrive(0, 0, 0);
    }

    // go until red line
    private void movetored(int numberOfRed) {
        elapsedTime.reset();
        redColorFound = 0;
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < numberOfRed ){
            boolean isRedFound = isRedColorFound();
            hardwarePushBot.mecanumDrive(0, FORWARD_SLOW, 0);
            //feedbackMovement.driveWithFeedback(hardwarePushBot,FORWARD_SLOW,0);
            if(isRedFound)
                redColorFound++;

        }

        hardwarePushBot.mecanumDrive(0,0,0);
        telemetry.addData("Color Red", hue);
        telemetry.addData("RedColorFound", redColorFound);
        telemetry.update();
    }

    // deposit WB and return





    /**
     * If Color Sensor detects Red Color. or white color
     *
     * @return
     */


    // color detection
    public boolean iswhitefound() {
        boolean found = false;
        double lightIntensity;
        //     lightIntensity = hardwarePushBot.rightColorSensor.alpha(); // total light luminosity


        Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        // CheckForRed is a hsv value check

        hue = hsvValues[0];
        saturation = hsvValues[1];
        value = hsvValues[2];

        telemetry.addData("Color White", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
        telemetry.addData("Color White value", value);
        telemetry.addData("Color White saturation", saturation);
        telemetry.update();

        if (value >= LIGHT_INTENSITY_WHITE) {
            found = true;
           sleep(20);
        }

        return found;
    }



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
