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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Red Left Target C FBM TS ", group="Wobble Goal")
public class WobbleGoal_RedLeft_TargetC_FBM_TS extends LinearOpMode {
    double RIGHT_SLOW = -0.4;
    double LEFT_SLOW = 0.4;
    double TIMEOUT_WG_WALL = 1;
    double STRAFE_TIME_RED_LEFT = 0.5;
    double STRAFE_TIME_RED_RIGHT = 0.5;
    double TIMEOUT_WG_TGC = 6.0;
    double LIGHT_INTENSITY_WHITE = 30;
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
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    double globalAngle;

    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;
    double heading;
    double integralError =0;
    double error =0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double STRAFE_LEFT = -0.3;
    double STRAFE_RIGHT = 0.3;


    HardwarePushBot hardwarePushBot = new HardwarePushBot();
   // FeedbackMovement feedbackMovement = new FeedbackMovement();

    @Override
    public void runOpMode() {

        initHardwareMap();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        isStop = false;


        while (opModeIsActive() && !isStop) {
            drivewWithFeedback_FBM(0.4,0,6);
            drivewWithFeedback_touchSensor(0.2,0,4);
            drivewWithFeedback_FBM_Colors(0,-0.3,5,"RED",1);

            isStop = true;
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
        hardwarePushBot.mapTouchSensor(hardwareMap);
        hardwarePushBot.setWheelDirection();

        hardwarePushBot.initializeImu(hardwareMap);
        //feedbackMovement.initIntegralError(0, hardwarePushBot);
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



 public boolean isColorFound(String colorString){
        switch(colorString) {
            case "RED":{
                return isRedColorFound();
            }
            case "WHITE":{
                return iswhitefound();
            }
            default:
                return false;
            //Add a case for blue
        }

 }

    public void drivewWithFeedback_FBM(double drive_power, double strafe_power, double timeOut){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method

        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()&& elapsedTime.seconds()<timeOut){
            heading = hardwarePushBot.getAngle();
            error = 0-heading; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
    }

    public void drivewWithFeedback_FBM_Colors(double drive_power, double strafe_power, double timeOut, String colorString, int numoftimes){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method
        elapsedTime.reset();
        int colorcount = 0;
        while (opModeIsActive()&& elapsedTime.seconds()<timeOut &&  colorcount < numoftimes){
            if(isColorFound((colorString))){
                colorcount++;
            }
            heading = hardwarePushBot.getAngle();
            error = 0-heading; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);

    }

    public void drivewWithFeedback_touchSensor(double drive_power, double strafe_power, double timeOut){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0);
        elapsedTime.reset();
        integralError=0;
        while ( hardwarePushBot.frontTouchSensor.getState() && opModeIsActive()&& elapsedTime.seconds()<timeOut){
            heading = hardwarePushBot.getAngle();
            error = 0-heading; // desrired - current heading is the error
            integralError = integralError + error*0.025;
            telemetry.addData("front touch sensor not touched ", hardwarePushBot.frontTouchSensor.getState());
            telemetry.update();
            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
        telemetry.addData("front touch sensor touched ", hardwarePushBot.frontTouchSensor.getState());
        telemetry.update();
    }


}
