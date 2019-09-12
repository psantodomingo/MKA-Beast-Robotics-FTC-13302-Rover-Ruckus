package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class Latch extends LinearOpMode{

 // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor latch1 = null;
    private DcMotor latch2 = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        latch1 = hardwareMap.get(DcMotor.class, "latch1");
        latch2 = hardwareMap.get(DcMotor.class, "latch2");

        latch1.setDirection(DcMotor.Direction.REVERSE);
        latch2.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double latchPower = 0;

            if(gamepad1.dpad_up){

                latchPower = -1;
            }
            else if(gamepad1.dpad_down){

                latchPower = 1;
            }

            latch1.setPower(latchPower);
            latch2.setPower(latchPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

