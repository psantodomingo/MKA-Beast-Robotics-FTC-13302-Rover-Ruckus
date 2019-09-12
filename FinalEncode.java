
package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//or
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import com.vuforia.Frame;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

@Autonomous(name="Final Auto", group ="Concept")

public class FinalEncode extends LinearOpMode {
    IntegratingGyroscope gyro;
    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    DigitalChannel digitalTouch;
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor1;
    NormalizedColorSensor colorSensor2;
    long start = System.currentTimeMillis();
    
    int test = 0;
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private Servo token = null;
    private DcMotor latch1 = null;
    private DcMotor latch2 = null;
    int flag = 0;
    int flag2 = 0;

    public static final String TAG = "Vuforia Navigation Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear  = hardwareMap.get(DcMotor.class, "leftRear");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("status", "Started" );
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "Ab6tv+D/////AAAAmSD12+Dx+0HxhLEnywpQXbOIBTLKRhuLvgD7uTYZxGEn0A80HWL9EqOGFJghV4jkfMc4CYh8dcK2s0IalwnnQNY0T+a884H4myt4WZl16cdzMXA/kwKKGZ+tPd7RjEYj4Kk5Lke6ocrVssdcSzHBoTVXPp1ZhMZJolzlO/syaW1gJEuBu3zDpniK7IlzMdB8m07UzZrwZ2un5jGBcaGRa3yDpWiFQxmtzCS3V33oPvvG5uHPCD4CtOtoGswuy7HCYVOJIaMFT4xNufDbRvXwg485cCs02HqX4S9f4sxuuT5JA4QYu+jY/OMjjl72vHLV64ScNcyhCvQAmLKuoxANvJXFi647tXVglUK60KMnKKHV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        token = hardwareMap.get(Servo.class, "token");
        token.setDirection(Servo.Direction.FORWARD);
        token.setPosition(0);
        
        

        waitForStart();
        //latchrot(9500);

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(1, true); //enables RGB565 format for the image
        locale.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

        /*To access the image: you need to iterate through the images of the frame object:*/

        Frame frame = locale.getFrameQueue().take(); //takes the frame at the head of the queue
        Image rgb = null;

        long numImages = frame.getNumImages();


        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == 1) {
                rgb = frame.getImage(i);
                telemetry.addData("Status", "Image stored" );
                telemetry.update();
                break;
            }//if
        }//for
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(),rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());
        int height = bm.getHeight();
        int width = bm.getWidth();
        for(int i=0; i<height && flag2 == 0; i++){
            for (int j = 0; j<width && flag2 == 0;j++){
                int color = bm.getPixel(j,i);
                int red = red(color);
                int green = green(color);
                int blue = blue(color);
                if (red>200 && blue<70 && green> 175){
                    telemetry.addData("status", red);
                    telemetry.addData("status", green);
                    telemetry.addData("status", blue);
                    telemetry.update();
                    if (j<width/3){
                        telemetry.addData("Status:", "1");
                        telemetry.update();
                        flag = 1;
                        flag2 = 1;
                        break;
                    }
                    if (j>(width/3)*2){
                        telemetry.addData("Status:", "3");
                        telemetry.update();
                        flag = 3;
                        flag2 = 1;
                        break;
                    } else{
                        telemetry.addData("Status:", "2");
                        telemetry.update();
                        flag = 2;
                        flag2 = 1;
                        break;
                    }
                }
            }
        }
        /*rgb is now the Image object that we’ve used in the video*/
        tilt(500);
        tiltleft(500);
        runforward(500);
        //crater();
        
        
    }


    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public void tilt(int rotate) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

double targetencpos = leftFront.getCurrentPosition() - rotate;
        while (leftFront.getCurrentPosition() > targetencpos) {
            telemetry.addData("status", leftFront.getCurrentPosition() );
            telemetry.update();
            double r = Math.hypot(0, 0);
            double robotAngle = Math.atan2(0, 0) - Math.PI / 4;
            double rightX = .1;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;


            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
        }
        leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
    }

   
        
    public void latchrot(int rotations){
        latch1 = hardwareMap.get(DcMotor.class, "latch1");
        latch2 = hardwareMap.get(DcMotor.class, "latch2");
        latch1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        latch2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        latch2.setDirection(DcMotor.Direction.REVERSE);
        latch1.setDirection(DcMotor.Direction.REVERSE);
        telemetry.addData("Status", latch2.getCurrentPosition());
        telemetry.update();
        double targetencpos = latch2.getCurrentPosition() - rotations;
        while(latch2.getCurrentPosition() > targetencpos){
            latch1.setPower(0.6);
            latch2.setPower(0.6);
            telemetry.addData("Status", latch2.getCurrentPosition());
            telemetry.update();
        }
        latch1.setPower(0);
        latch2.setPower(0);
        

    }
   
    public void runforward(int rotate){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
 rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        double targetencpos = leftFront.getCurrentPosition() - rotate;

        while(leftFront.getCurrentPosition() > targetencpos){
            telemetry.addData("status", leftFront.getCurrentPosition() );
            telemetry.update();
            double r = Math.hypot(0, .15);
                double robotAngle = Math.atan2(.15, 0) - Math.PI / 4;
                double rightX = 0;
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;


                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                                rightRear.setPower(v4);
        }
    
         leftFront.setPower(0);
                rightFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);

    
    }
     public void tiltleft(int rotate) {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        double targetencpos = leftFront.getCurrentPosition() + rotate;
        while (leftFront.getCurrentPosition() < targetencpos) {
            telemetry.addData("status", leftFront.getCurrentPosition() );
            telemetry.update();
            double r = Math.hypot(0, 0);
            double robotAngle = Math.atan2(0, 0) - Math.PI / 4;
            double rightX = -.1;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;


            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
        }
        leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        
    }



    }
