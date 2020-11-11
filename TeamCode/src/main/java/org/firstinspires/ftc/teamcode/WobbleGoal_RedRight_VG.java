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
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Moochi_lah", group="Wobble Goal")
//@Disabled
public class WobbleGoal_RedRight_VG extends LinearOpMode {

    /* Declare OpMode members. */
// The objects below are classes that help us connect to the robot hardware

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftRearDrive = null;
    private DcMotor rightRearDrive = null;
    private DcMotor linearSlide = null;
    private Servo leftHand = null;
    private Servo rightHand = null;
    private Servo capstoneLeft = null;
    private Servo capstoneRight = null;
    private Servo frontArm = null;
    private ColorSensor sensorColor = null;
    private DistanceSensor sensorDistance = null;
    private RevColorSensorV3 leftColorSensor=null;
    private RevColorSensorV3 rightColorSensor=null;

    // Globals + constants are defined here
    //declare variables with  scope of entire class
    double  leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower =0;
    double rightRearPower = 0;
    double error =0;
    Orientation lastAngles = new Orientation();
    double globalAngle;

    BNO055IMU imu;

    double PROP_GAIN = 0.015;
    double INT_GAIN = 0.015;
    double DER_GAIN = 0.015;
    double delta_turn;

    double RIGHT_SLOW = 0.2;
    double TIMEOUT_WG_WALL = 0.5;
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

// Here we establish connection between our Java Classes defined above and the Robot Configuration defined in the Robot Controller.
// If the Robot Controller name does not match the name given below in the green text, there will be a run time error when you hit init in the RC phone
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front");
        leftRearDrive  = hardwareMap.get(DcMotor.class, "left_rear");
        rightRearDrive = hardwareMap.get(DcMotor.class, "right_rear");
        linearSlide = hardwareMap.get(DcMotor.class,"linear_slide");
        leftHand = hardwareMap.get(Servo.class,"left_hand");
        rightHand = hardwareMap.get(Servo.class,"right_hand");
        frontArm = hardwareMap.get(Servo.class,"front_arm");
        capstoneLeft=hardwareMap.get(Servo.class,"capstone_left");
        capstoneRight=hardwareMap.get(Servo.class,"capstone_right");
        sensorColor = hardwareMap.get(ColorSensor.class, "color_sensor_front");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor_front");
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class,"color_sensor_front");

        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Define IMU Variables

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
        resetAngle(); // set heading to zero

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // Test 1: 2019 - code is running fine.
/*        mecanumDrive(0,0.5,0);
        elapsedTime.reset();
        while(opModeIsActive() && (elapsedTime.seconds()<3.0)){
            error = 0 - getAngle();
            delta_turn = PROP_GAIN*error;
            mecanumDrive(0,0.5,-    delta_turn);
        }
        mecanumDrive(0,0,0);*/

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
        while (opModeIsActive() && (elapsedTime.seconds()<TIMEOUT_WG_TGC) && redColorFound == 3 ){
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

    // Methods~^^
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
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
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
}
