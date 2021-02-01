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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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

@Autonomous(name="Ultimate Auto Encoder", group="Pushbot")
//@Disabled
public class UltimateAutoEncoder extends LinearOpMode {

    OpenCvInternalCamera phoneCam;
    SkystoneDeterminationPipeline pipeline;

    //Rev Hub 1
    private DcMotor frontLeft; // left side driving mechanism(motor 0)
    private DcMotor frontRight; // right side driving mechanism (motor 1)
    private DcMotor backLeft; // back left driving mechanism (motor 2)
    private DcMotor backRight; // back right driving mechanism (motor 3)
    private DcMotor intake; // intake motor to suck up disks
    private DcMotor conveyor; // runs conveyor belt to launch area
    private DcMotor launch; // spins wheel to launch disks
    private DcMotor arm; // moves arm forward and back
    private Servo launchgate; // controls disks from launch area
    private Servo intakerelease; // allows intake to drop in to place
    private Servo claw; // grabs wobble

    /* Declare OpMode members. */
    //HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     DRIVE_SPEED = 0.7;
    static final double     TURN_SPEED  = 0.5;
    static final double     STOP        = 0;
    static final double     clawopen = 0.75;
    static final double     clawclose = 0.9;
    static final double     intakerestriction = 0.9;
    static final double     launchspeed = 0.78;
    static final double     armspeed = 0.8;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeveRest 40
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // Mecanum wheel diameter
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        //pipeline = new SkystoneDeterminationPipeline();
        pipeline = new SkystoneDeterminationPipeline();

        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        //robot.init(hardwareMap);
        frontLeft  = (DcMotor)hardwareMap.get("frontLeft");
        frontRight = (DcMotor)hardwareMap.get("frontRight");
        backLeft = (DcMotor)hardwareMap.get("backLeft");
        backRight = (DcMotor)hardwareMap.get("backRight");
        intake = (DcMotor)hardwareMap.get("intake");
        conveyor = (DcMotor)hardwareMap.get("conveyor");
        launch = (DcMotor)hardwareMap.get("launch");
        arm = (DcMotor)hardwareMap.get("arm");

        launchgate = (Servo)hardwareMap.get("launchgate");
        intakerelease = (Servo)hardwareMap.get("intakerelease");
        claw = (Servo)hardwareMap.get("claw");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        launch.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        conveyor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setPower(STOP);
        frontLeft.setPower(STOP);
        backLeft.setPower(STOP);
        backRight.setPower(STOP);
        intake.setPower(STOP);
        conveyor.setPower(STOP);
        launch.setPower(STOP);
        arm.setPower(STOP);
        launchgate.setPosition(0);
        intakerelease.setPosition(0.4);
        claw.setPosition(0.9);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            sleep(500);

            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                // Drive right to wall
                encoderDrive(DRIVE_SPEED,18,-18,-18,18,5);
                sleep(500);
                // Drive forward to launch line
                encoderDrive(DRIVE_SPEED,48,48,48,48,10);
                sleep(500);
                // Launch line adjustments
                encoderDrive(DRIVE_SPEED,5,6,5,6,5);
                // Launch disks
                launch.setPower(launchspeed);
                launchgate.setPosition(1);
                sleep(2000);
                conveyor.setPower(0.5);
                sleep(5000);
                launchgate.setPosition(STOP);
                launch.setPower(STOP);
                conveyor.setPower(STOP);
                // Drop wobble
                encoderDrive(DRIVE_SPEED,40,40,40,40,5);
                arm.setPower(armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawopen);
                sleep(500);
                arm.setPower(-armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawclose);
                sleep(500);
                // Navigate back to cover launch line
                encoderDrive(DRIVE_SPEED,-30,-30,-30,-30,5);
                sleep(500);
            } else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                // Drive right to wall
                encoderDrive(DRIVE_SPEED,18,-18,-18,18,5);
                sleep(500);
                // Drive forward to launch line
                encoderDrive(DRIVE_SPEED,48,48,48,48,10);
                sleep(500);
                // Launch line adjustments
                encoderDrive(DRIVE_SPEED,5,6,5,6,5);
                // Launch disks
                launch.setPower(launchspeed);
                launchgate.setPosition(1);
                sleep(2000);
                conveyor.setPower(0.5);
                sleep(5000);
                launchgate.setPosition(STOP);
                launch.setPower(STOP);
                conveyor.setPower(STOP);
                // Drop wobble
                encoderDrive(DRIVE_SPEED,-28,28,28,-28,5);
                encoderDrive(DRIVE_SPEED,11,11,11,11,5);
                arm.setPower(armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawopen);
                sleep(500);
                arm.setPower(-armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawclose);
                sleep(500);
            } else {
                // Drive right to wall
                encoderDrive(DRIVE_SPEED,18,-18,-18,18,5);
                sleep(500);
                // Drive forward to Target Zone A
                encoderDrive(DRIVE_SPEED,48,48,48,48,10);
                sleep(500);
                // Navigate to launch line
                encoderDrive(DRIVE_SPEED,5,6,5,6,5);
                // Launch disks
                launch.setPower(launchspeed);
                launchgate.setPosition(1);
                sleep(2000);
                conveyor.setPower(0.5);
                sleep(5000);
                launchgate.setPosition(STOP);
                launch.setPower(STOP);
                conveyor.setPower(STOP);
                // Drop wobble
                encoderDrive(DRIVE_SPEED,-6,-6,-6,-6,5);
                arm.setPower(armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawopen);
                sleep(500);
                arm.setPower(-armspeed);
                sleep(1000);
                arm.setPower(0);
                claw.setPosition(clawclose);
                sleep(500);
                // Navigate left to clear wobble
                encoderDrive(DRIVE_SPEED,-18,18,18,-18,5);
                // Navigate forward to cover launch line
                encoderDrive(DRIVE_SPEED,12,12,12,12,5);
                sleep(500);
            }
            sleep(600000);
        }
    }


    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(90,197);

        static final int REGION_WIDTH = 30;
        static final int REGION_HEIGHT = 20;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 140;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;


        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches,
                             double timeoutS) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newfrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newbackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newbackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newfrontLeftTarget);
            frontRight.setTargetPosition(newfrontRightTarget);
            backLeft.setTargetPosition(newbackLeftTarget);
            backRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget, newbackLeftTarget, newbackRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontRight.setPower(STOP);
            frontLeft.setPower(STOP);
            backLeft.setPower(STOP);
            backRight.setPower(STOP);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
