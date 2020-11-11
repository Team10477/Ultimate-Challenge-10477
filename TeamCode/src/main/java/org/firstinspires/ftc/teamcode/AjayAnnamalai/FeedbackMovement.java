package org.firstinspires.ftc.teamcode.AjayAnnamalai;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FeedbackMovement {
    AutoDriveByTime_Linear_Gyro_SG robotControls = new AutoDriveByTime_Linear_Gyro_SG();
    HardwarePushbot robot = new HardwarePushbot();
    FeedbackMovement feedbackMovement = new FeedbackMovement();
    // The IMU sensor object
    BNO055IMU imu;

    double leftFrontPower;
    double rightFrontPower;
    double leftRearPower;
    double rightRearPower;

    Orientation angles;
    Acceleration gravity;
    Orientation lastAngles = new Orientation();

    double globalAngle;
    double heading;
    double integralError = 0;
    double error = 0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double STRAFE_LEFT = -0.3;
    double STRAFE_RIGHT = 0.3;

    public void initializeImu(HardwareMap hardwareMap) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }

    public void driveWithAngle(double drive, double strafe, double turn, org.firstinspires.ftc.AjayAnnamalai.HardwarePushbot robot) {

        leftFrontPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
        rightFrontPower = Range.clip(drive - turn + strafe, -1.0, 1.0);
        leftRearPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
        rightRearPower = Range.clip(drive - turn - strafe, -1.0, 1.0);

        robot.leftFrontWheel.setPower(leftFrontPower);
        robot.rightFrontWheel.setPower(rightFrontPower);
        robot.leftBackWheel.setPower(leftRearPower);
        robot.rightBackWheel.setPower(rightRearPower);
    }

    public void driveWithFeedback(org.firstinspires.ftc.AjayAnnamalai.HardwarePushbot robot, double drivePower, double strafePower) {
        heading = getAngle();
        error = (0 - heading);
        integralError = integralError + (0 - heading) * 0.05;
        deltaTurn = error * GAIN_PROP + integralError * GAIN_INT;

        driveWithAngle(drivePower, strafePower, deltaTurn, robot);
    }

    public void initIntegralError(double power, HardwarePushbot robot) {
        integralError = 0;
        error = 0;
        driveWithAngle(0, power, 0, robot);
        //  resetAngle();
    }


    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
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

    public void turnWithGyro(int degrees, double speed) {
        while (getAngle() != 90) {
            if (getAngle() < 90) {
                robotControls.StrafeRight(false, 0, 0.3);
            } else if (getAngle() > 90) {
                robotControls.StrafeRight(true, 0, 0.3);
            }
        }
    }

    public void driveToColor(float[] HSVValues, double speed) {
        float[] currentHSVvalues = {0F, 0F, 0F};
        while (currentHSVvalues != HSVValues) {
            feedbackMovement.driveWithFeedback(robot, 0.3, 0);
            Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, currentHSVvalues);
        }
    }
}
