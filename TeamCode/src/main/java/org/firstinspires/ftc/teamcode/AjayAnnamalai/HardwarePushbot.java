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

package org.firstinspires.ftc.teamcode.AjayAnnamalai;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Right front drive motor: "right_front"
 * Motor channel:  Left front drive motor: "left_front"
 * Motor channel:  Left back drive motor: "left_rear"
 * Motor channel:  Right back drive motor: "right_rear"
 */
public class HardwarePushbot {
    /* Public OpMode members. */

    public Servo pickupArm = null;
    public DigitalChannel touchSensorPickupArm = null;

    public RevColorSensorV3 colorSensorFront = null;
    public DcMotor leftBackWheel = null;
    public DcMotor rightBackWheel = null;
    public DcMotor leftFrontWheel = null;
    public DcMotor rightFrontWheel = null;


    RevColorSensorV3 colorSensor = null;    // Hardware Device Object
    RevColorSensorV3 colorSensorRight = null;    // Hardware Device Object

    HardwareMap hwMap = null;

    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        defineMotors(hwMap);

        colorSensor = hwMap.get(RevColorSensorV3.class, "color_sensor");

        colorSensorRight = hwMap.get(RevColorSensorV3.class, "color_sensor_right");
        pickupArm = hwMap.get(Servo.class, "front_arm");
        touchSensorPickupArm = hwMap.get(DigitalChannel.class, "touch_sensor_arm");

        colorSensorFront = hwMap.get(RevColorSensorV3.class, "color_sensor_front");

        setWheelDirectionForward();

        setWheelPower(0);

        // Set all motors to run without encoders.
        setNoEncoder();


    }

    public void setNoEncoder() {
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void defineMotors(HardwareMap hwMap) {
        leftBackWheel = hwMap.get(DcMotor.class, "left_rear");
        rightBackWheel = hwMap.get(DcMotor.class, "right_rear");
        leftFrontWheel = hwMap.get(DcMotor.class, "left_front");
        rightFrontWheel = hwMap.get(DcMotor.class, "right_front");

    }

    public void setWheelDirectionReverse() {
        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark mototor
        leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setWheelDirectionForward() {
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark mototor
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotor.Direction.FORWARD);
    }

    // Set all motors to given power
    public void setWheelPower(double power) {
        rightFrontWheel.setPower(power);
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(power);
        rightBackWheel.setPower(power);
    }

    // Set all motors to given power
    public void setWheelPowerBackward(double power) {
        rightFrontWheel.setPower(-power);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(-power);
        rightBackWheel.setPower(-power);
    }


    // Set all motors to given power to strafe
    public void setWheelPowerForSide(double power) {
        rightFrontWheel.setPower(-power);
        leftFrontWheel.setPower(power);
        leftBackWheel.setPower(-power);
        rightBackWheel.setPower(power);
    }

    // Strafe Adjustment because robot tends to move backward during strafe.
    public void setWheelPowerForSideWithDelta(double power, double deltaP) {
        rightFrontWheel.setPower(power * deltaP);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(power * deltaP);
        rightBackWheel.setPower(-power);
    }

    // Set all motors to given power to strafe
    public void setWheelPowerTurnRight(double power) {
        rightFrontWheel.setPower(power);
        leftFrontWheel.setPower(-power);
        leftBackWheel.setPower(-power);
        rightBackWheel.setPower(power);
    }

    // Set all motors to zero power
    public void stopWheels() {
        rightFrontWheel.setPower(0);
        leftFrontWheel.setPower(0);
        leftBackWheel.setPower(0);
        rightBackWheel.setPower(0);
    }


}

