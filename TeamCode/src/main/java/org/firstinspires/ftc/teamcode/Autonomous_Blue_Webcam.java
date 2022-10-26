package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Stack;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Rect;


@Disabled
@Autonomous(name="Autonomous Blue Position 1 Webcam", group="Blue Pos")

public class Autonomous_Blue_Webcam extends LinearOpMode {


    private DcMotorEx MotorRight;
    private DcMotorEx MotorLeft;
    private DcMotorEx frontRightWheel;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx Arm;
    private DcMotorEx ArmJoint;
    private CRServo Claw;
    private DcMotorEx TopWheel;

    static double PI=3.141592;
    static double CIRCUMFERENCE = 76/25.4 * PI;
    static double GEAR_3_RATIO = 2.89;
    static double GEAR_4_RATIO = 3.61;
    static double GEAR_5_RATIO = 5.23;
    static double COUNTS_PER_IN_DRIVE = 28*GEAR_3_RATIO*GEAR_4_RATIO*GEAR_3_RATIO/CIRCUMFERENCE;


    static double COUNT_PER_DEGREE_ARM   = 28*GEAR_3_RATIO*GEAR_4_RATIO*GEAR_5_RATIO*(125.0/30.0)*(90.0/45.0)/360;
    static double COUNT_PER_DEGREE_ARMJOINT   = 28*GEAR_3_RATIO * GEAR_4_RATIO * GEAR_5_RATIO/360;

    static double COUNT_PER_360_ROTATE=6300;
    static double COUNT_PER_360_ROTATE_SPEED   = 25.5;
    OpenCvWebcam webcam;
    static int Element_Position;
    static int detect;



    // Function to Move in straight line

    // Distance in inches
    // Speed in inches/sec
    public void move(double distance, double speed, String status)
    {
        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorRight.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
        MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorRight.setVelocity(  COUNTS_PER_IN_DRIVE * speed ); // Set Velocity is in Ticks per Second

        frontLeftMotor.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setVelocity( COUNTS_PER_IN_DRIVE * speed);

        frontRightWheel.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setVelocity(  COUNTS_PER_IN_DRIVE * speed);


        MotorLeft.setTargetPosition((int)(distance * COUNTS_PER_IN_DRIVE));
        MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorLeft.setVelocity( COUNTS_PER_IN_DRIVE * speed );


        while (opModeIsActive() &&
                (MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()) ){
            telemetry.addData("Status", status);
            telemetry.addData("Motor Left", MotorLeft.getCurrentPosition() - MotorLeft.getTargetPosition());
            telemetry.addData("Motor Right", MotorRight.getCurrentPosition() - MotorRight.getTargetPosition());
            telemetry.addData("Motor Front Right", frontRightWheel.getCurrentPosition() - frontRightWheel.getTargetPosition());
            telemetry.addData("Motor Front Left", frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition());
            telemetry.update();
        }
    }

    // Rotate around center of Robot
    // Degrees and degrees per sec
    public void rotate(double angle, double speed, String status)
    {

        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorRight.setTargetPosition( (int)((angle/360)*COUNT_PER_360_ROTATE));
        MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorRight.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );

        MotorLeft.setTargetPosition( (int)((-angle/360)*COUNT_PER_360_ROTATE));
        MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorLeft.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );

        frontRightWheel.setTargetPosition( (int)((angle/360)*COUNT_PER_360_ROTATE));
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );

        frontLeftMotor.setTargetPosition( (int)((-angle/360)*COUNT_PER_360_ROTATE));
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setVelocity(speed*COUNT_PER_360_ROTATE_SPEED );



        while (opModeIsActive() &&
                (MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()) ){
            telemetry.addData("Status", status);
            telemetry.addData("Diff", frontRightWheel.getCurrentPosition() - frontRightWheel.getTargetPosition());
            telemetry.update();

        }

    }
    /*
        public void strafe(double distance, double angle, double speed, String status)
        {


            MotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            MotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double y = Math.cos(angle*PI/180)*speed;
            double x = Math.sin(angle*PI/180)*speed;


            MotorRight.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE ); // Set Velocity is in Ticks per Secon
            frontLeftMotor.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE);
            frontRightWheel.setVelocity((y-x)*COUNTS_PER_IN_DRIVE );
            MotorLeft.setVelocity( (y-x)*COUNTS_PER_IN_DRIVE);

            telemetry.addData("Status", status);
            telemetry.addData("Time to move", (distance/speed));
            telemetry.update();

            sleep((long)(distance/speed)*1000);

            MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    */
    public void strafe(double distance, double angle, double speed, String status)
    {


        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double y = Math.cos(angle*PI/180)*speed;
        double x = Math.sin(angle*PI/180)*speed;
        double dy = Math.cos(angle*PI/180)*distance;
        double dx = Math.sin(angle*PI/180)*distance;

        MotorRight.setTargetPosition((int) ((dy + dx)*COUNTS_PER_IN_DRIVE) ); // Set Velocity is in Ticks per Secon
        frontLeftMotor.setTargetPosition( (int)((dy + dx)*COUNTS_PER_IN_DRIVE));
        frontRightWheel.setTargetPosition((int)((dy-dx)*COUNTS_PER_IN_DRIVE) );
        MotorLeft.setTargetPosition( (int)((dy-dx)*COUNTS_PER_IN_DRIVE));

        MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorRight.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE ); // Set Velocity is in Ticks per Secon
        frontLeftMotor.setVelocity( (y + x)*COUNTS_PER_IN_DRIVE);
        frontRightWheel.setVelocity((y-x)*COUNTS_PER_IN_DRIVE );
        MotorLeft.setVelocity( (y-x)*COUNTS_PER_IN_DRIVE);

        while (opModeIsActive() &&
                (MotorLeft.isBusy() || MotorRight.isBusy() || frontLeftMotor.isBusy() || frontRightWheel.isBusy()) ){
            telemetry.addData("Status", status);
            telemetry.addData("Motor Left", MotorLeft.getCurrentPosition() - MotorLeft.getTargetPosition());
            telemetry.addData("Motor Right", MotorRight.getCurrentPosition() - MotorRight.getTargetPosition());
            telemetry.addData("Motor Front Right", frontRightWheel.getCurrentPosition() - frontRightWheel.getTargetPosition());
            telemetry.addData("Motor Front Left", frontLeftMotor.getCurrentPosition() - frontLeftMotor.getTargetPosition());
            telemetry.update();
        }


    }

    public void UpdateData(){
        telemetry.addData("ArmJointTargetPos", ArmJoint.getCurrentPosition()/COUNT_PER_DEGREE_ARMJOINT);
        telemetry.addData("ArmTargetPos", Arm.getCurrentPosition()/COUNT_PER_DEGREE_ARM);
        telemetry.update();
    }
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        MotorLeft = hardwareMap.get(DcMotorEx.class, "MotorLeft");
        MotorRight = hardwareMap.get(DcMotorEx.class, "MotorRight");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightWheel = hardwareMap.get(DcMotorEx.class, "frontRightWheel");
        ArmJoint = hardwareMap.get(DcMotorEx.class, "arm joint");
        Arm = hardwareMap.get(DcMotorEx.class, "arm");
        TopWheel = hardwareMap.get(DcMotorEx.class, "topwheelmotor");
        Claw = hardwareMap.get(CRServo.class, "claw");

        // Reverse one of the drive motors.
        frontRightWheel.setDirection(DcMotorEx.Direction.REVERSE);
        ArmJoint.setDirection(DcMotorEx.Direction.REVERSE);


        MotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webcam.setPipeline(new CustomElementPositionPipeline());


        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(1280, 960, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addLine("Camera Opening Error !!!!!");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        //waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            if (Element_Position == 1) {
                telemetry.addData("Element Position", "Left");
            }
            if (Element_Position == 2) {
                telemetry.addData("Element Position", "Center");
            }
            if (Element_Position == 3) {
                telemetry.addData("Element Position", "Right");
            }
            telemetry.addData("Element Position Num", Element_Position);


            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                /*
                 * IMPORTANT NOTE: calling stopStreaming() will indeed stop the stream of images
                 * from the camera (and, by extension, stop calling your vision pipeline). HOWEVER,
                 * if the reason you wish to stop the stream early is to switch use of the camera
                 * over to, say, Vuforia or TFOD, you will also need to call closeCameraDevice()
                 * (commented out below), because according to the Android Camera API documentation:
                 *         "Your application should only have one Camera object active at a time for
                 *          a particular hardware camera."
                 *
                 * NB: calling closeCameraDevice() will internally call stopStreaming() if applicable,
                 * but it doesn't hurt to call it anyway, if for no other reason than clarity.
                 *
                 * NB2: if you are stopping the camera stream to simply save some processing power
                 * (or battery power) for a short while when you do not need your vision pipeline,
                 * it is recommended to NOT call closeCameraDevice() as you will then need to re-open
                 * it the next time you wish to activate your vision pipeline, which can take a bit of
                 * time. Of course, this comment is irrelevant in light of the use case described in
                 * the above "important note".
                 */
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            //sleep(100);
        }


        waitForStart();


        if (opModeIsActive()) {
            if (Element_Position == 1) {
                telemetry.addData("Element Position", "Left");
            }
            if (Element_Position == 2) {
                telemetry.addData("Element Position", "Center");
            }
            if (Element_Position == 3) {
                telemetry.addData("Element Position", "Right");
            }
            telemetry.addData("Element Position Num", Element_Position);
            telemetry.update();
            UpdateData();
            // Grab Initial block
            Claw.setPower(-.2);sleep(1200);

             /*
              while (opModeIsActive() && Arm.isBusy() ){
                telemetry.addData("Arm Be Move", Arm.getVelocity());
                telemetry.update();
              }
            */

            //Use variable Element_Position to determine arm height
            //MEASURE DISTANCES
            if (Element_Position == 1) {

                ArmJoint.setTargetPosition((int) (127 * COUNT_PER_DEGREE_ARMJOINT));
                ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmJoint.setVelocity(100 * COUNT_PER_DEGREE_ARMJOINT);
                /*while (opModeIsActive() && ArmJoint.isBusy()){
                    telemetry.addData("Armjoint Be Move", ArmJoint.getVelocity());
                    telemetry.update();
                }*/
                sleep(3000);

                //Lower Level
                Arm.setTargetPosition((int)(-70* COUNT_PER_DEGREE_ARM));
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setVelocity(100*COUNT_PER_DEGREE_ARM );
                telemetry.addData("Arm Be Move", Arm.getVelocity());
                telemetry.update();
                strafe(-25, 55, 15,"");

            }
            else if (Element_Position == 2) {
                //Middle Level
                ArmJoint.setTargetPosition((int) (58.15 * COUNT_PER_DEGREE_ARMJOINT));
                ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmJoint.setVelocity(100 * COUNT_PER_DEGREE_ARMJOINT);
                sleep(3000);
                Arm.setTargetPosition((int)(0* COUNT_PER_DEGREE_ARM));
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setVelocity(100*COUNT_PER_DEGREE_ARM );


                strafe(-27, 55, 15,"");
            }
            else {
                //Top Level
                ArmJoint.setTargetPosition((int) (63.15 * COUNT_PER_DEGREE_ARMJOINT));
                ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmJoint.setVelocity(100 * COUNT_PER_DEGREE_ARMJOINT);
                sleep(2000);

                Arm.setTargetPosition((int)(15.38* COUNT_PER_DEGREE_ARM));
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setVelocity(75*COUNT_PER_DEGREE_ARM );

                strafe(-29, 50, 15,"");
            }

            UpdateData();
            // If we want to wait until the armjoint is done.. uncomment this
            telemetry.addData("ArmJointTargetPos", ArmJoint.getTargetPosition());

            while (opModeIsActive() && Arm.isBusy() ){
                telemetry.addData("Arm Joint Be Move", ArmJoint.getVelocity());
                telemetry.addData("ArmJointTargetPos", ArmJoint.getTargetPosition());
                telemetry.update();
            }

            Claw.setPower(1); sleep(750);
            Claw.setPower(0);

            //Going Back from Dropping the Block
            if (Element_Position ==1){
                move(12, 15, "roboreverse");
            }
            else if (Element_Position ==2){
                move(16, 15, "roboreverse");
            }
            else{
                move(19, 15, "roboreverse");
            }

            rotate(-95, 45, "TARGET LOCKED"); //sleep(2000);


            Arm.setTargetPosition((int)(25* COUNT_PER_DEGREE_ARM));
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setVelocity(100*COUNT_PER_DEGREE_ARM);
            ArmJoint.setTargetPosition((int)(125* COUNT_PER_DEGREE_ARMJOINT));
            ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJoint.setVelocity(100*COUNT_PER_DEGREE_ARMJOINT);

            UpdateData();
            //sleep(1500);
            move(-66,9999,"Robot go BRRRRRRRRRRR");

            // To Roll a Duck.. Uncomment this
   /*
      move(-4, 20, "roboreverse");
      rotate(180, 45, "TARGET LOCKED");
     //Start from Po1
      strafe (65,-90,20,"");
      strafe (2,-90,10,"");
      move(10, 5, "roboreverse");
      TopWheel.setPower (.5);sleep(2500);
      TopWheel.setPower (0);
     */

     /*
      // To part in the small parking.. Uncomment this and comment next block
      move(-18,20,"Robot go BRRRRRRRRR");
      Arm.setTargetPosition((int)(70* COUNT_PER_DEGREE_ARM));
      Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Arm.setVelocity(60*COUNT_PER_DEGREE_ARM );
      */
      /*
      // Go to Warehouse
      strafe (20,90,15,"");
      rotate(-105, 45, "TARGET LOCKED"); //sleep(2000);
      move(100,100,"Robot go BRRRRRRRRR");
*/
        }
    }



    public static class CustomElementPositionPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the team element position
         */
        public enum ElementPosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar ORANGE = new Scalar(255, 100, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(109,98);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181,98);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253,98);
        static final Point REGION1_CENTER_ANCHOR_POINT = new Point(100,200);
        static final Point REGION2_CENTER_ANCHOR_POINT = new Point(610,200);
        static final Point REGION3_CENTER_ANCHOR_POINT = new Point(1100,200);
        static final int REGION_WIDTH = 80;
        static final int REGION_HEIGHT = 80;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_CENTER_ANCHOR_POINT.x,
                REGION1_CENTER_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_CENTER_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_CENTER_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_CENTER_ANCHOR_POINT.x,
                REGION2_CENTER_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_CENTER_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_CENTER_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_CENTER_ANCHOR_POINT.x,
                REGION3_CENTER_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_CENTER_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_CENTER_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition position = EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition.LEFT;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    ORANGE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            /*
             * Find the max of the 3 averages
             */
            int maxOneTwo = Math.min(avg1, avg2);
            int max = Math.min(maxOneTwo, avg3);

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */
            if(max == avg1) // Was it from region 1?
            {
                position = EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition.LEFT; // Record our analysis
                Element_Position = 1;
                detect = 1;

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill

            }
            else if(max == avg2) // Was it from region 2?
            {
                position = EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition.CENTER; // Record our analysis
                Element_Position = 2;
                detect = 1;
                /*
                 * Draws a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
            else if(max == avg3) // Was it from region 3?
            {
                position = EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition.RIGHT; // Record our analysis
                Element_Position = 3;
                detect = 1;

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }
          /*  if (Element_Position == 0){
                detect=0;
            }
*/
            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */
        public EasyOpenCV_Test.CustomElementPositionPipeline.ElementPosition getAnalysis()
        {
            return position;
        }


    }

}

