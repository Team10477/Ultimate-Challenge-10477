package org.firstinspires.ftc.teamcode.AjayAnnamalai;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//ths code is for target zone A
//  Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, Hsv_Values);

public class BlueA extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();
    MyColorSensor MyColorSensor = new MyColorSensor();
    FeedbackMovement feedbackMovement = new FeedbackMovement();
    AutoDriveByTime_Linear_Gyro_SG AutoDrive = new AutoDriveByTime_Linear_Gyro_SG();
    float Hsv_Values[] = {0F, 0F, 0F};

    //fix the hsv values and adjust it for blue
    float HsvBlue[] = {240f, 0f, 0f};

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MyColorSensor.enableColorSensor(robot.colorSensorFront, hardwareMap);
        MyColorSensor.enableColorSensor(robot.colorSensor, hardwareMap);
        feedbackMovement.initializeImu(hardwareMap);


        //step one
        AutoDrive.StrafeRight(false, 750, 0.5);

        //step two
        while (Hsv_Values != HsvBlue) {
            feedbackMovement.driveWithFeedback(robot, 0, 1);
            Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, Hsv_Values);
        }

        //step 2.5
        AutoDrive.MoveForward(400, 0.5);


        //step 3
        AutoDrive.SetArmPosition(0);
    }


}
