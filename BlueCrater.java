
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

@Autonomous(name="Blue Crater", group ="Concept")

public class BlueCrater extends LinearOpMode {
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

        
        

        waitForStart();
        latchrot(10300);

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
        if(flag ==3){telemetry.addData("status", "script finished");
        telemetry.addData("Flag",flag);
        telemetry.update();
        tilt(62);
        colorrun();
        crater();
        }
        //crater(); 
        
        
        if(flag ==2 || flag == 0){
        tilt(80);
        colorrun();
        crater();
        }
        //crater();*/
        
         
        if(flag ==1){
        tilt(105);
        colorrun();
        crater();
        }
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

        boolean lastResetState = false;
        boolean curResetState = false;

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.
        if(test == 0){
            telemetry.log().add("Gyro Calibrating. Do Not Move!");
        modernRoboticsI2cGyro.calibrate();

        // Wait until the gyro calibration is complete
        while (!isStopRequested() && modernRoboticsI2cGyro.isCalibrating()) {
            telemetry.update();
            sleep(10);
        }
        test++;
        }
        // Wait for the start button to be pressed
        // If the A and B buttons are pressed just now, reset Z heading.
        
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        modernRoboticsI2cGyro.getHeading();
        modernRoboticsI2cGyro.getIntegratedZValue();
        telemetry.addData("Status", modernRoboticsI2cGyro.getHeading());
        telemetry.addData("Status", modernRoboticsI2cGyro.getIntegratedZValue());
        telemetry.update();
        while (modernRoboticsI2cGyro.getIntegratedZValue() > -rotate && opModeIsActive()) {
            double r = Math.hypot(0, 0);
            double robotAngle = Math.atan2(0, 0) - Math.PI / 4;
            double rightX = .2;
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

    public void crater(){
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

        boolean lastResetState = false;
        boolean curResetState = false;

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.

        // Wait for the start button to be pressed
        // If the A and B buttons are pressed just now, reset Z heading.
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        modernRoboticsI2cGyro.getHeading();
        modernRoboticsI2cGyro.getIntegratedZValue();
        telemetry.addData("Status", modernRoboticsI2cGyro.rawY());
        telemetry.update();
        while (modernRoboticsI2cGyro.rawX() > -1200 && opModeIsActive()) {
            double r = Math.hypot(0, 0.4);
            double robotAngle = Math.atan2(0.4, 0) - Math.PI / 4;
            double rightX = 0;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            
            leftFront.setPower(v1);
            rightFront.setPower(v2);
            leftRear.setPower(v3);
            rightRear.setPower(v4);
            telemetry.addData("Status", modernRoboticsI2cGyro.rawX());
            telemetry.update();
        }
    }
        public void touchsense() {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            rightFront = hardwareMap.get(DcMotor.class, "rightFront");
            rightRear = hardwareMap.get(DcMotor.class, "rightRear");
            leftRear = hardwareMap.get(DcMotor.class, "leftRear");
            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightRear.setDirection(DcMotor.Direction.REVERSE);            digitalTouch = hardwareMap.get(DigitalChannel.class, "sensor_digital");

            // set the digital channel to input.
            digitalTouch.setMode(DigitalChannel.Mode.INPUT);

            while (digitalTouch.getState() == true && opModeIsActive()) {
                double r = Math.hypot(0, .3);
                double robotAngle = Math.atan2(.3, 0) - Math.PI / 4;
                double rightX = 0;
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;

                telemetry.addData("Status", digitalTouch.getState());
                telemetry.update();
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }
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
    public void colorrun(){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "sensor_color1");
        if (colorSensor1 instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor1).enableLight(true);
        }
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        token = hardwareMap.get(Servo.class, "token");
        token.setDirection(Servo.Direction.FORWARD);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int color = colors.toColor();
        NormalizedRGBA colors1 = colorSensor1.getNormalizedColors();
        int color1 = colors1.toColor();
        float max = Math.max(Math.max(Math.max(colors.red, colors.green), colors.blue), colors.alpha);
        colors.red   /= max;
        colors.green /= max;
        colors.blue  /= max;
        color = colors.toColor();
        colors1.red   /= max;
        colors1.green /= max;
        colors1.blue  /= max;
        color1 = colors1.toColor();

        while((Color.blue(color)<200 && Color.blue(color1)<200) && opModeIsActive()){
            telemetry.addData("Status",Color.red(color));
            telemetry.addData("Status",Color.red(color1));
            telemetry.update();
            double r = Math.hypot(0, .3);
                double robotAngle = Math.atan2(.3, 0) - Math.PI / 4;
                double rightX = 0;
                double v1 = r * Math.cos(robotAngle) + rightX;
                double v2 = r * Math.sin(robotAngle) - rightX;
                double v3 = r * Math.sin(robotAngle) + rightX;
                double v4 = r * Math.cos(robotAngle) - rightX;


                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
                colors = colorSensor.getNormalizedColors();
                colors.red   /= max;
                colors.green /= max;
                colors.blue  /= max;
                color = colors.toColor();
                colors1 = colorSensor1.getNormalizedColors();
                colors1.red   /= max;
                colors1.green /= max;
                colors1.blue  /= max;
                color1 = colors1.toColor();
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

        boolean lastResetState = false;
        boolean curResetState = false;

        // Get a reference to a Modern Robotics gyro object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "gyro");
        // A similar approach will work for the Gyroscope interface, if that's all you need.

        // Start calibrating the gyro. This takes a few seconds and is worth performing
        // during the initialization phase at the start of each opMode.

        // Wait for the start button to be pressed
        // If the A and B buttons are pressed just now, reset Z heading.
        
        modernRoboticsI2cGyro.resetZAxisIntegrator();
        modernRoboticsI2cGyro.getHeading();
        modernRoboticsI2cGyro.getIntegratedZValue();
        telemetry.addData("Status", modernRoboticsI2cGyro.getHeading());
        telemetry.addData("Status", modernRoboticsI2cGyro.getIntegratedZValue());
        telemetry.update();
        while (modernRoboticsI2cGyro.getIntegratedZValue() < rotate && opModeIsActive()) {
            double r = Math.hypot(0, 0);
            double robotAngle = Math.atan2(0, 0) - Math.PI / 4;
            double rightX = -.2;
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
