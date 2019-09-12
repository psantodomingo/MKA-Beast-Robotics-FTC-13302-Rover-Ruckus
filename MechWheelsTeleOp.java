package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class MechWheelsTeleOp extends LinearOpMode{

    
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor collector = null;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotor latch1 = null;
    private DcMotor latch2 = null;
    private Servo token = null;
    private Servo collectorExtension1 = null;
    private Servo collectorExtension2 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //collector  = hardwareMap.get(DcMotor.class, "collector");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");

        latch1 = hardwareMap.get(DcMotor.class, "latch1");
        latch2 = hardwareMap.get(DcMotor.class, "latch2");
        /*token = hardwareMap.get(Servo.class, "token");
        collectorExtension1 = hardwareMap.get(Servo.class, "collectorExtension");
        collectorExtension2 = hardwareMap.get(Servo.class, "collectorExtension");*/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //collector.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        
        latch1.setDirection(DcMotor.Direction.REVERSE);
        latch2.setDirection(DcMotor.Direction.REVERSE);
        //token.setDirection(Servo.Direction.FORWARD);
        //collectorExtension1.setDirection(Servo.Direction.FORWARD);
        //collectorExtension2.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //token.setPosition(0);

            double power = 0;

            if (gamepad1.a){

                power = 1;
            }
            else if(gamepad1.b){

                power = -1;
            }

            //collector.setPower(power);

            //Above is collector code

 
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;


            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);


            //Above is Wheel Code

            double latchPower = 0;

            if(gamepad1.dpad_up){

                latchPower = 1;
            }
            else if(gamepad1.dpad_down){

                latchPower = -1;
            }

            latch1.setPower(latchPower);
            latch2.setPower(latchPower);

            //Above is Latch

            double collectorExtension = 0;


            if (gamepad1.right_bumper){

                collectorExtension = 0;
            }

            else if(gamepad1.left_bumper){

                collectorExtension = 1;

            }

            //collectorExtension1.setPosition(collectorExtension);
            //collectorExtension2.setPosition(collectorExtension);

            //Above is collector extension

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

