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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous
public class RightAprilTagAuto extends LinearOpMode
{
    //April Tag Declarations
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

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    String TagIdentified = "Center";

    AprilTagDetection tagOfInterest = null;


    //Drive Declarations
    private DcMotor         LeftBack   = null;
    private DcMotor         LeftFront  = null;
    private DcMotor         RightFront = null;
    private DcMotor         RightBack = null;
    private DcMotor         Lift = null;
    private Servo           RightServo = null;
    private Servo           LeftServo = null;
    private ElapsedTime     runtime = new ElapsedTime();

    //Encoder Declarations
    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.2;
    static final double LIFT_UP = 5;
    static final double LIFT_DOWN = -5;
    static final double OPEN = 0.5;
    static final double CLOSED = 0.5;



    @Override
    public void runOpMode()
    {

        //Drive Variables
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LeftFront  = hardwareMap.get(DcMotor.class, "LeftFront");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        Lift = hardwareMap.get(DcMotor.class, "RightFront");
        RightServo = hardwareMap.get(Servo.class, "ClawRight");
        LeftServo = hardwareMap.get(Servo.class, "ClawLeft");


        //Direction and Encoder Set
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //OpenCV Init
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 18)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                        TagIdentified = "Center"
                    } else if(tag.id == 17){
                        tagOfInterest = tag;
                        tagFound = true;
                        TagIdentified = "Left";
                    }
                    else if(tag.id == 19){
                        tagOfInterest = tag;
                        tagFound = true;
                        TagIdentified = "Right";
                    }

                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
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
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        encoderDrive(0.2, 4, 4, -4, RightFront, 0.2);
        Lift.setPower(0.3);
        encoderDrive(0.2, 10, 10, 10, RightFront, 0.2);
        encoderDrive(0.2, 0, 4, 4, LeftFront, 0.2);
        RightServo.setPosition(1);
        LeftServo.setPosition(1);
        Lift.setPower(-0.2);
        if(TagIdentified == "Left"){
            encoderDrive(0.2, 0, -4, -4, LeftFront, 0.2);
            encoderDrive(0.2, -2, 0, 0, LeftFront, 0.2);
        }
        if(TagIdentified == "Center"){
            encoderDrive(0.2, 0, -4, -4, LeftFront, 0.2);
            encoderDrive(0.2, -1, -1, -1, LeftFront, 0.2);
        }
        if(TagIdentified == "Right"){
            encoderDrive(0.2, -6, -6, -6, LeftFront, 0.2);
            encoderDrive(0.2, 0, -2, -2, LeftFront, 0.2);
        }



        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
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



    public void encoderDrive(double speed,  double LeftFrontInches, double RightBackInches, double RightFrontInches, String LeftBackSet, double timeoutS) {
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;
        String LeftBackSet;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newRightBackTarget = RightBack.getCurrentPosition() + (int)(RightBackInches * COUNTS_PER_INCH);
            newLeftFrontTarget = LeftFront.getCurrentPosition() + (int)(LeftFrontInches * COUNTS_PER_INCH);
            newRightFrontTarget = RightFront.getCurrentPosition() + (int)(RightFrontInches * COUNTS_PER_INCH);
            RightBack.setTargetPosition(newRightBackTarget);
            LeftFront.setTargetPosition(newLeftFrontTarget);
            RightFront.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(LeftBackSet.isBusy){
                LeftBack.setPower(0.2);
            }
            while (!LeftBackSet.isBusy) {
                LeftBack.setPower(0);
            }


            runtime.reset();
            RightBack.setPower(Math.abs(speed));
            LeftFront.setPower(Math.abs(speed));
            RightFront.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LeftFront.isBusy() && LeftBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftBackTarget,  newRightBackTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftBack.getCurrentPosition(), LeftFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftFront.setPower(0);
            RightFront.setPower(0);
            RightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.


        }
    }
}
