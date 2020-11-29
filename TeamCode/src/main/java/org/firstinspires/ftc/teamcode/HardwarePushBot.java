package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class HardwarePushBot {

    /**
     * Four mecanum wheels.
     **/
    public DcMotor leftBackWheel = null;
    public DcMotor rightBackWheel = null;
    public DcMotor leftFrontWheel = null;
    public DcMotor rightFrontWheel = null;
    public DcMotor ringIntake = null;

    /**
     * Two color Sensors
     */
    public RevColorSensorV3 leftColorSensor = null;
    public RevColorSensorV3 rightColorSensor = null;

    /**
     * Touch Sensors
     **/
    public DigitalChannel leftTouchSensor = null;
    public DigitalChannel rightTouchSensor = null;
    public DigitalChannel frontTouchSensor = null;

    /**
     * Servos
     **/
    public Servo wobbleGoalArm;

    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftRearPower = 0;
    double rightRearPower = 0;

    public void init(HardwareMap hwMap) {
        mapWheels(hwMap);
        mapWobbleArmServo(hwMap);
        mapTouchSensor(hwMap);
        mapColorSensor(hwMap);
        mapRingIntake(hwMap);
    }

    public void mapWheels(HardwareMap hwMap) {
        leftBackWheel = hwMap.get(DcMotor.class, "left_rear");
        rightBackWheel = hwMap.get(DcMotor.class, "right_rear");
        leftFrontWheel = hwMap.get(DcMotor.class, "left_front");
        rightFrontWheel = hwMap.get(DcMotor.class, "right_front");
    }

    public void mapRingIntake(HardwareMap hwMap) {
        ringIntake = hwMap.get(DcMotor.class, "ring_intake");
    }
    public void mapWobbleArmServo(HardwareMap hwMap) {
        wobbleGoalArm = hwMap.get(Servo.class, "servo"); // Check servo config. in RC
    }

    public void mapTouchSensor(HardwareMap hwMap) {
        leftTouchSensor = hwMap.get(DigitalChannel.class, "touch_sensor");  // Check servo config. in RC
        rightTouchSensor = hwMap.get(DigitalChannel.class, "touch_sensor_right");  // Check servo config. in RC
        frontTouchSensor = hwMap.get(DigitalChannel.class, "touch_sensor_front");  // Check servo config. in RC
    }

    public void mapColorSensor(HardwareMap hwMap) {
        leftColorSensor = hwMap.get(RevColorSensorV3.class, "color_sensor");  // Check servo config. in RC
        rightColorSensor = hwMap.get(RevColorSensorV3.class, "color_sensor_right");  // Check servo config. in RC

    }

    public void mecanumDrive(double drive, double strafe, double turn) {
        leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        leftRearPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightRearPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
        setWheelPower(leftFrontPower, rightFrontPower, leftRearPower, rightRearPower);
    }

    public void setWheelPower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftBackWheel.setPower(leftRearPower);
        rightBackWheel.setPower(rightRearPower);
    }

    public void setWheelDirection() {
        leftFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        leftBackWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightBackWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setPosition(double position) {
        wobbleGoalArm.setPosition(position);
    }

    public void enableColorSensor(RevColorSensorV3 colSensor, HardwareMap hwMap) {
        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        colSensor.enableLed(true);
    }



}
