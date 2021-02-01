package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;


@TeleOp(name = "StormComp20_21", group = "storm")

public class StormComp20_21
        extends OpMode {
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
    //private SensorColor color; // detects color on bottom of robot

    //Claw variables declaration
    double i = 0.6;

    boolean leftBumperWasPressed = false;
    boolean rightBumperWasPressed = false;

    @Override
    public void init() {
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

        //color = (SensorColor)hardwareMap.get("color");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        conveyor.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        launchgate.setPosition(0);
        intakerelease.setPosition(0.4);
        claw.setPosition(0.9);
    }

    @Override
    public void loop() {

        //Define variables
        double clawopen = 0.75;
        double clawclose = 0.9;
        double intakerestriction = 0.9;
        double launchspeed = 0.78;
        double armspeed = 0.7;



        // The amount of power to each wheel, set by the left and right gamepad sticks
        backLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        backRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        frontLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        frontRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

        // Define intake at back of robot
        intake.setPower(gamepad1.left_trigger*intakerestriction);
        intake.setPower(-gamepad1.right_trigger);

        // Define conveyor
        conveyor.setPower(gamepad2.left_trigger);

        // Define launch
        launch.setPower(launchspeed);

        // Define Launch Gate
        launchgate.setPosition(gamepad2.left_stick_y);

        // Define Intake Release
        intakerelease.setPosition(gamepad2.right_stick_y + 0.4);

        // Define Claw
        if (gamepad2.left_bumper)
            claw.setPosition(clawopen);
        if (gamepad2.right_bumper)
            claw.setPosition(clawclose);

        // Define Arm
        arm.setPower(gamepad2.right_stick_y*armspeed);
/*        if (gamepad2.dpad_up)
            arm.setPower(armspeed);
        if (gamepad2.dpad_down)
            arm.setPower(-armspeed);
        if (gamepad2.dpad_left)
            arm.setPower(0);
        if (gamepad2.dpad_right)
            arm.setPower(0);

 */


/*        // Define hook
        if (gamepad1.a)
                hook.setPosition(open);
        if (gamepad1.b)
                hook.setPosition(close);

        // Define clawGrab
        clawGrab.setPosition(gamepad2.left_trigger);

        //Define clawRotate
        if (gamepad2.left_bumper) {
            if(!leftBumperWasPressed) i+= 0.25;
            leftBumperWasPressed = true;
        }
        else leftBumperWasPressed = false;

        if (gamepad2.right_bumper) {
            if(!rightBumperWasPressed) i-= 0.25;
            rightBumperWasPressed = true;
        }
        else rightBumperWasPressed = false;

        clawRotate.setPosition(i);

        // Define lift
        lift.setPower(gamepad2.right_stick_y);
*/

    }
            }

