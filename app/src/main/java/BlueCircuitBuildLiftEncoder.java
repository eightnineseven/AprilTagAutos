/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;







@Autonomous
public class BlueCircuitBuildLiftEncoder extends LinearOpMode
{
    private DcMotor         LeftBack = null;
    private DcMotor         RightBack = null;
    private DcMotor         LeftFront  = null;
    private DcMotor         RightFront = null;
    private DcMotor         Lift = null;
    private Servo           RightServo = null;
    private Servo           LeftServo = null;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;
    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_18 = 18; // Tag ID 18 from the 36h11 family
    int ID_TAG_17 = 17;
    int ID_TAG_19 = 19;

    String TagIdentified = "Center";

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        RightServo = hardwareMap.get(Servo.class, "ClawRight");
        LeftServo = hardwareMap.get(Servo.class, "ClawLeft");

        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftServo.setDirection(Servo.Direction.FORWARD); // Left side looking from back
        RightServo.setDirection(Servo.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_18) {
                        tagOfInterest = tag;
                        tagFound = true;
                        TagIdentified = "Center";
                        break;
                    } else if (tag.id == ID_TAG_19) {
                        tagOfInterest = tag;
                        TagIdentified = "Right";
                        tagFound = true;
                        break;
                    } else if (tag.id == ID_TAG_17) {
                        tagOfInterest = tag;
                        TagIdentified = "Left";
                        tagFound = true;
                        break;
                        //17=Left   18=Middle    19=Right
                    }

                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            if (tagOfInterest.pose.x <= 20) {
                // do something
            } else if (tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                // do something else
            } else if (tagOfInterest.pose.x >= 50) {
                // do something else
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {
            //push cone to taped area
            //mecanum 0.8 tiles
            encoderDrive(0.2, -7, -7, 7, 5, 0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            //go to stack
            //straight 2 tiles
            encoderDrive(0.2, 10, 10, 10,  0,0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            //turn 90*
            encoderDrive(0.2, 0, -5, -5,  0,0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(-0.2);
            }
            LeftBack.setPower(0);

            //straight 0.7 tiles
            encoderDrive(0.2, 2, 2, 2,  0,0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            //grab cone off stack
            RightServo.setPosition(0.85);
            LeftServo.setPosition(0.85);

            //lift for low terminal
            encoderDrive(0.2, 0, 0, 0,  12,0.2);
            LeftBack.setPower(0);

            //straight 2 tiles
            encoderDrive(0.2, -12, -12, -12,  0,0.2);
            while(LeftFront.isBusy){
                LeftBack.setPower(-0.2);
            }
            LeftBack.setPower(0);

            //sideways to terminal, no turn as to conserve time while still being consistent. Turns = bad
            //mecanum 1.6 tiles
            encoderDrive(0.2, -10, -10, 10,  0,0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            //drop cone
            RightServo.setPosition(1);
            LeftServo.setPosition(1);

            //First cone stacked, going for second
            //mecanum 1.6 tiles
            encoderDrive(0.2, 10, 10, -10,  0,0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(-0.2);
            }
            LeftBack.setPower(0);

            //straight 2 tiles
            encoderDrive(0.2, 12, 12, 12,  -12, 0.2);
            while(LeftFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            RightServo.setPosition(0.85);
            LeftServo.setPosition(0.85);

            //cone grabbed
            //added to allow robot to lift the lift as to not tip stack.
            encoderDrive(0.2, 0, 0, 0,  30, 0.2);
            LeftBack.setPower(0);

            //go to far, high junction for shortest circuit
            //straight 3 tiles
            encoderDrive(0.2, -12, -12, -12,  0, 0.2);
            while(LeftFront.isBusy){
                LeftBack.setPower(-0.2);
            }
            LeftBack.setPower(0);

            //mecanum 0.5 tiles
            encoderDrive(0.2, -4, -4, 4, 0,  0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(0.2);
            }
            LeftBack.setPower(0);

            RightServo.setPosition(0.85);
            LeftServo.setPosition(0.85);
            //cone dropped

            //mecanum 0.5 tiles
            encoderDrive(0.2, 4, 4, -4, 0,  0.2);
            while(RightFront.isBusy){
                LeftBack.setPower(-0.2);
            }
            LeftBack.setPower(0);

            //parking based on April Tag detection.
            //straight one tiles
            if(TagIdentified == "Left"){
                encoderDrive(0.2, 6, 6, 6, 0, 0.2);
                while(LeftFront.isBusy){
                    LeftBack.setPower(0.2);
                }
                LeftBack.setPower(0);
            }
            //straight two tiles
            if(TagIdentified == "Center"){
                encoderDrive(0.2, 12, 12, 12, 0, 0.2);
                while(LeftFront.isBusy){
                    LeftBack.setPower(0.2);
                }
                LeftBack.setPower(0);
            }
            //straight three tiles
            if(TagIdentified == "Right"){
                encoderDrive(0.2, 15, 15, 15, 0, 0.2);
                while(LeftFront.isBusy){
                    LeftBack.setPower(0.2);
                }
                LeftBack.setPower(0);
            }
            sleep(20000000);

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void encoderDrive(double speed,  double LeftFrontInches, double RightBackInches, double RightFrontInches, double LiftSet, double timeoutS) {
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLiftSet;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = (int) ((double) LeftFront.getCurrentPosition() / COUNTS_PER_INCH + (int)(LeftFrontInches * COUNTS_PER_INCH));
            newRightFrontTarget = (int) ((double) RightFront.getCurrentPosition() / COUNTS_PER_INCH + (int)(RightFrontInches * COUNTS_PER_INCH));
            newRightBackTarget = (int) ((double) RightBack.getCurrentPosition() / COUNTS_PER_INCH + (int)(RightBackInches * COUNTS_PER_INCH));
            newLiftSet = (int) (COUNTS_PER_INCH * (int)(LiftSet));
            LeftFront.setTargetPosition(newLeftFrontTarget * 3);
            RightFront.setTargetPosition(newRightFrontTarget * 3);
            RightBack.setTargetPosition(newRightBackTarget * 3);


            // Turn On RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));
            RightBack.setPower(Math.abs(speed);
            Lift.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS)) {

                // Display it for the driver.
                telemetry.update();
            }

            // Stop all motion;
            LeftFront.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);
            Lift.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.


        }
    }
}

