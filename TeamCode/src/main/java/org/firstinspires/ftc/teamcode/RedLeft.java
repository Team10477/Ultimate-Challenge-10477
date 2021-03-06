package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

@Autonomous(name="Right left", group = "Wobble Goal")

public class RedLeft extends LinearOpMode {

        private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
        private static final String LABEL_QUAD_ELEMENT = "Quad";
        private static final String LABEL_SINGLE_ELEMENT = "Single";

        private static final String VUFORIA_KEY =
                "AZvVY5H/////AAABmVIJC2J+Bk8dmED6q+xzKkUsIvfAZMxMAHooL0K1pItNOLCvp0Mq1vBnpcHiPjwlLYt5OOVpnoyE72D5Ku8iR1ahgCo9Le6ymYWRR/No45AHVGMbfpnYxmMchxYfGn/otWPMzvohNI2lyeJwmumFgHIO9nAJ7YH86HptFVWynm16S2ENaAUfBOeKxqPABdc7i8PCe7r/PMeY2gVSAp9/53S3Gi9PGvv4f2wCtlSQChSAsbdglIuy91gLnHHzR0w1itEjtRvSnronkqr/2P9xNfWcp8tN7duysWh6eQlUhAWerhD05mvmBorePfpmCfS2sWn4FTMdzR3vZZxypY1NgOx7OvYSEhPaapYcX8ydsBOb";

        private VuforiaLocalizer vuforia;
        private TFObjectDetector tfod;


        private int targetZone = 0;

        private ElapsedTime elapsedTime = new ElapsedTime();
        private ElapsedTime loopTime = new ElapsedTime();

        HardwarePushBotNew hardwarePushBot = new HardwarePushBotNew();

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 8;
        int redColorFound = 0;
        int whitecolorfound = 0;
        float hue = 0, saturation = 0, value = 0;
        boolean isStop = false;
        BNO055IMU imu;
        double heading;
        double integralError = 0;
        double error = 0;
        double deltaTurn = 0;
        double GAIN_PROP = 0.015;
        double GAIN_INT = 0.015;
        double strafeForSpead = 0.6;
        double driveForSpeed = 0.5;
        double strafeForColorSensor = 0.3;
        double driveForColorSensor = 0.2;
        boolean revOn = false;

        @Override
        public void runOpMode() throws InterruptedException {
            initVuforia();
            initTfod();
            activateTfod();

            if(opModeIsActive()){
                identifyRings();
                telemetry.addData("Target Zone", targetZone);


            }

        }
        //Target Zone code
    public void shootingPosition(){
      revOn();
      driveMecanum(driveForSpeed,0,2.0);
      revOn = true;
      drivewWhite(driveForColorSensor,0,3.0);
      driveMecanum(-0.4,0,0.7);
      driveMecanum(0,0.3, 1);
      shoot();
      revOn = false;
        }
    public void targetZoneA(){
    drivewRed(0,0,0 );
    driveMecanum(-0.3,0,1);

    }
    public void targetZoneB(){

    }
    public void targetZoneC(){

    }


// code provided by FTC
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }
        private void initTfod() {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD_ELEMENT, LABEL_SINGLE_ELEMENT);
        }
        public void activateTfod() {
            /**
             * Activate TensorFlow Object Detection before we wait for the start command.
             * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
             **/
            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
                tfod.setZoom(2.5, 16.0/9.0);
            }
        }
        public void identifyRings() {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0 ) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        targetZone = 1;
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                targetZone = 2;
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                targetZone = 3;
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }

                    telemetry.update();
                }
            }
        }
        public void deactivateTfod() {
            if (tfod != null) {
                tfod.shutdown();
            }
        }

//movement code plus methods for the movement code

        public void driveMecanum(double drive_power, double strafe_power, double timeOut){
            hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method

            elapsedTime.reset();
            integralError=0;
            while (opModeIsActive()&& elapsedTime.seconds()<timeOut){
                heading = hardwarePushBot.getAngle();
                error =heading - 0; // desrired - current heading is the error
                integralError = integralError + error*0.025;

                hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
            }
            hardwarePushBot.mecanumDrive(0,0.0,0.0);
        }
        public void drivewRed(double drive_power, double strafe_power, double timeOut){
            hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method
            elapsedTime.reset();
            integralError=0;
            while (opModeIsActive()&& elapsedTime.seconds()<timeOut && !isRedColorFound()){

                heading = hardwarePushBot.getAngle();
                error = heading - 0; // desrired - current heading is the error
                integralError = integralError + error*0.025;

                hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
            }
            hardwarePushBot.mecanumDrive(0,0.0,0.0);
        }
        public void drivewWhite(double drive_power, double strafe_power, double timeOut){
        hardwarePushBot.mecanumDrive(drive_power,strafe_power,0.0); // pass the parameters to a mecanumDrive method
        elapsedTime.reset();
        integralError=0;
        while (opModeIsActive()&& elapsedTime.seconds()<timeOut && !isWhitefound()){

            heading = hardwarePushBot.getAngle();
            error = heading - 0; // desrired - current heading is the error
            integralError = integralError + error*0.025;

            hardwarePushBot.mecanumDrive(drive_power,strafe_power,-(error*GAIN_PROP+integralError*GAIN_INT)); // the multiplication of 0.015 is a gain to make the turn power small (not close to 1, which is maximum)
        }
        hardwarePushBot.mecanumDrive(0,0.0,0.0);
    }

        public boolean isRedColorFound() {
            boolean found = false;

            Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                    (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                    (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            // CheckForRed is a hsv value check

            hue = hsvValues[0];

            if((hue < 60 || hue > 320) ){
                found = true;
                sleep(50);
            }

            return found;
        }
        public boolean isWhitefound() {
            boolean found = false;
            double lightIntensity;
            //     lightIntensity = hardwarePushBot.rightColorSensor.alpha(); // total light luminosity


            Color.RGBToHSV((int) (hardwarePushBot.rightColorSensor.red() * SCALE_FACTOR),
                    (int) (hardwarePushBot.rightColorSensor.green() * SCALE_FACTOR),
                    (int) (hardwarePushBot.rightColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            // CheckForRed is a hsv value check

            value = hsvValues[2];



            if (value >= 38) {
                found = true;
            }

            return found;
        }
//depositing
    public void lowerArm(){

    }
    public void releaseArm(){

    }
//shooting
public void revOn(){
while(revOn = true){
    hardwarePushBot.shootingWheel.setPower(1.0);
}
}
public void shoot() {
    for (int i = 0; i < 3; i++) {
        hardwarePushBot.shootingTrigger.setPosition(90);
        hardwarePushBot.shootingTrigger.setPosition(0);
        sleep(500);
    }
}

    }

