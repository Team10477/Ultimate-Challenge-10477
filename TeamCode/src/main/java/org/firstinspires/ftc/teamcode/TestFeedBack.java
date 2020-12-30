package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test FeedBack")
public class TestFeedBack extends LinearOpMode {

    private ElapsedTime elapsedTime = new ElapsedTime();

    HardwarePushBot hardwarePushBot = new HardwarePushBot();

    double heading;
    double integralError =0;
    double error =0;
    double deltaTurn = 0;
    double GAIN_PROP = 0.015;
    double GAIN_INT = 0.015;
    double STRAFE_LEFT = -0.3;
    double STRAFE_RIGHT = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() ){
            //Strafe left for 2 seconds
            drivewWithFeedback_FBM(0,0.3,2);

            //drive forward for 2 seconds
            drivewWithFeedback_FBM(0.3,0,2);
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

        hardwarePushBot.initializeImu(hardwareMap);

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
}
