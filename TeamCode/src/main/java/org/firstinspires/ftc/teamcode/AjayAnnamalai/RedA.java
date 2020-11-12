package org.firstinspires.ftc.teamcode.AjayAnnamalai;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//ths code is for target zone A
//  Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, Hsv_Values);

public class RedA extends LinearOpMode {
    HardwarePushbot robot = new HardwarePushbot();
    MyColorSensor MyColorSensor = new MyColorSensor();
    FeedbackMovement feedbackMovement = new FeedbackMovement();
    AutoDriveByTime_Linear_Gyro_SG AutoDrive = new AutoDriveByTime_Linear_Gyro_SG();
    float Hsv_Values[] = {0F, 0F, 0F};
    float HsvRed[] = {0f, 0f, 100f};
    float HsvWhite[] = {0f, 0f, 100f};

    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MyColorSensor.enableColorSensor(robot.colorSensorFront, hardwareMap);
        MyColorSensor.enableColorSensor(robot.colorSensor, hardwareMap);
        feedbackMovement.initializeImu(hardwareMap);


        //step one
        feedbackMovement.driveToColor(HsvWhite, 0.4);

        //step two
        feedbackMovement.turnWithGyro(90, 0.35);

        //step three

        feedbackMovement.driveToColor(HsvRed, 0.4);

        //step four
        AutoDrive.SetArmPosition(0);

        /* while (Hsv_Values != HsvRed) {
            feedbackMovement.driveWithFeedback(robot, 0, 1);
            Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, Hsv_Values);
        }*/

          /* while (Hsv_Values != HsvWhite) {
            feedbackMovement.driveWithFeedback(robot, 0, 1);
            Color.RGBToHSV(robot.colorSensorFront.red() * 8, robot.colorSensorFront.green() * 8, robot.colorSensorFront.blue() * 8, Hsv_Values);
        }*/
    }
}
