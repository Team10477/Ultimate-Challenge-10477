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
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Red Left Target A FBM", group="Wobble Goal")
public class WobbleGoal_RedLeft_TargetA_FBM extends LinearOpMode {
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

        elapsedTime.reset();


        while (opModeIsActive() && !isStop) {

            // step 1: strafe left half a square
            strafeLeft();
            //Step 2: move straight until white line
            sleep(500);
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

        initializeImu(hardwareMap);
        //feedbackMovement.initIntegralError(0, hardwarePushBot);
    }


    /**
     * Strafe Right to the Wall.
     */
    // add touch sensor later, to improve the performance.
    private void strafeLeft() {
        initIntegralError(LEFT_SLOW, hardwarePushBot);
     // while (elapsedTime.time() < 4000) {
           // hardwarePushBot.mecanumDrive(0, LEFT_SLOW, 0);
           driveWithFeedback(hardwarePushBot,0,LEFT_SLOW, telemetry);

           sleep((long) (STRAFE_TIME_RED_LEFT*1000));
     //   }
       //hardwarePushBot.mecanumDrive(0, 0, 0);
        driveWithFeedback(hardwarePushBot,0,0);
    }

    // Strafe right to red line
    private void strafe_to_redRight(int numberofred){
        elapsedTime.reset();
        redColorFound = 0;
        initIntegralError(RIGHT_SLOW, hardwarePushBot);
        while ((elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < numberofred ){
            boolean isRedFound = isRedColorFound();
            driveWithFeedback(hardwarePushBot,0,RIGHT_SLOW);
           // hardwarePushBot.mecanumDrive(0, RIGHT_SLOW, 0);


           // feedbackMovement.initIntegralError(RIGHT_SLOW, hardwarePushBot);
           // feedbackMovement.driveWithFeedback(hardwarePushBot,0,RIGHT_SLOW);
            if(isRedFound)
                redColorFound++;

        }

    //    feedbackMovement.initIntegralError(RIGHT_SLOW, hardwarePushBot);
        driveWithFeedback(hardwarePushBot,0,0);
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
      //  elapsedTime.reset();
        redColorFound = 0;
        whitecolorfound = 0;
        boolean iswhitefound = false;
       // initIntegralError(RIGHT_SLOW, hardwarePushBot);
        while ( !iswhitefound) {
            driveWithFeedback(hardwarePushBot,FORWARD_SLOW,0, telemetry);
            iswhitefound = iswhitefound();
         }
        driveWithFeedback(hardwarePushBot,0,0);
     //  hardwarePushBot.mecanumDrive(0, 0, 0);
        isStop = true;
        telemetry.addData("Color white", hue);
        telemetry.addData("WhiteColorFound", redColorFound);
        telemetry.update();
    }

    // strafe right
    private void strafeRight() {
        elapsedTime.reset();

            hardwarePushBot.mecanumDrive(0, RIGHT_SLOW, 0);
            //feedbackMovement.driveWithFeedback(hardwarePushBot,0,RIGHT_SLOW);
        sleep((long) (STRAFE_TIME_RED_RIGHT*1000));
        hardwarePushBot.mecanumDrive(0, 0, 0);
    }

    // go until red line
    private void movetored(int numberOfRed) {
        elapsedTime.reset();
        redColorFound = 0;
        while (redColorFound < numberOfRed ){
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
    public void initializeImu(HardwareMap hardwareMap) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void driveWithAngle(double drive, double strafe, double turn, HardwarePushBot robot){

        leftFrontPower   = Range.clip(drive+turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(drive-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(drive+turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(drive-turn-strafe , -1.0, 1.0);

        robot.setWheelPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower );
    }

    public void driveWithFeedback(HardwarePushBot robot, double drivePower, double strafePower, Telemetry telemetry) {
        heading = getAngle();
        telemetry.addData("IMU Angle: ", heading);
        telemetry.update();
        error = (0-heading);
        integralError = integralError + (0-heading)*0.05;
        deltaTurn = error*GAIN_PROP + integralError*GAIN_INT;

        driveWithAngle(drivePower,strafePower, deltaTurn, robot);
    }
    public void driveWithFeedback(HardwarePushBot robot, double drivePower, double strafePower) {
        heading = getAngle();
        error = (0-heading);
        integralError = integralError + (0-heading)*0.05;
        deltaTurn = error*GAIN_PROP + integralError*GAIN_INT;

        driveWithAngle(drivePower,strafePower, deltaTurn, robot);
    }
    public void initIntegralError(double power, HardwarePushBot robot) {
        integralError=0;
        error = 0;
        driveWithAngle(0, power,0, robot);
        //  resetAngle();
    }
}
//Literally my life is beaches every single night, messy buns and christmas lights, literally my life is, literally my life is
// does anyone read these?