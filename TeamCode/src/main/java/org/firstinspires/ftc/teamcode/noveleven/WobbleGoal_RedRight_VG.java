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

package org.firstinspires.ftc.teamcode.noveleven;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Moochi_lah", group="Wobble Goal")
public class WobbleGoal_RedRight_VG extends LinearOpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;

    private RevColorSensorV3 leftColorSensor=null;
    private RevColorSensorV3 rightColorSensor=null;

    // Globals + constants are defined here
    //declare variables with  scope of entire class
    double  leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower =0;
    double rightRearPower = 0;
    double error =0;
    double globalAngle;

    BNO055IMU imu;

    double PROP_GAIN = 0.015;
    double INT_GAIN = 0.015;
    double DER_GAIN = 0.015;
    double delta_turn;

    double RIGHT_SLOW = -0.4;
    double TIMEOUT_WG_WALL = 1;
    double TIMEOUT_WG_TGC = 6.0;
    private ElapsedTime elapsedTime = new ElapsedTime();
    private ElapsedTime loopTime = new ElapsedTime();

    float hsvValues[] = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 8;
    int redColorFound = 0;
    float hue =0, saturation=0, value=0;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_right");

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step 1: strafe to the wall
        // add touch sensor later, to improve the performance.
        elapsedTime.reset();
        while (opModeIsActive() && (elapsedTime.seconds()<TIMEOUT_WG_WALL) ) {
            mecanumDrive(0, RIGHT_SLOW, 0);
        }
        mecanumDrive(0,0,0);
        //Step 2: move to Target Zone "C"

        elapsedTime.reset();
        redColorFound = 0;
        while (opModeIsActive() && (elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound < 3 ){
            mecanumDrive(0.4, 0 ,0);
            Color.RGBToHSV((int) (rightColorSensor.red() * SCALE_FACTOR),
                    (int) (rightColorSensor.green() * SCALE_FACTOR),
                    (int) (rightColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            // CheckForRed is a hsv value check

            hue = hsvValues[0];
            saturation = hsvValues[1];
            value = hsvValues[2];

            telemetry.addData("Color Red", String.valueOf(hue), String.valueOf(saturation), String.valueOf(value));
            telemetry.update();
            if(hue < 60 && hue > 320 && saturation > 0.5 && value > 0.5){
                redColorFound++;
            }
        }
        mecanumDrive(0,0,0);

    }

    public void mecanumDrive(double drive, double strafe, double turn){
        leftFrontPower   = Range.clip(drive+turn-strafe , -1.0, 1.0);
        rightFrontPower  = Range.clip(drive-turn+strafe , -1.0, 1.0);
        leftRearPower    = Range.clip(drive+turn+strafe , -1.0, 1.0);
        rightRearPower   = Range.clip(drive-turn-strafe , -1.0, 1.0);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftRearDrive.setPower(leftRearPower);
        rightRearDrive.setPower(rightRearPower);
    }

}
