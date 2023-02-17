package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Left_RR_TP_Vision", group="Left")
public class AutoUsingDetector_Left_RR_Vision extends LinearOpMode {
    private OpenCvCamera webcam;


    RobotArm BeepArm= new RobotArm();

    //Create New Robot based on RobotBase
    //RobotBase Beep = new RobotBase();

    // Dont really need to change this.
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Change these values to move the little square/region where we check what color we see
    private static double RegionCenterX = 875; // Distance in pixels from the Left
    private static double RegionCenterY = 340; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;


    // Change these values for the PID and autocorrection
    public static double Kp = 0.0004;
    public static int clawCenter = 690;
    public static double x = .75; // Max power during Strafe
    public static int pixelMargin = 35;
    private static int currentPos;

    private  NanoClock clock;
    private double currentAdjustTime;
    private double adjustTimeLimit=6;
    private double distAdjust=0;

    //height before a drop
    private static double preDropH = 5;

    private BBBDetector_Color.ElementPosition ParkingPos;

    public static Vector2d coneStack = new Vector2d(-62.75, -13.5);
    public static Vector2d shortPole = new Vector2d(-50,-19);

    public static Vector2d p1 = new Vector2d(-40,-65); // Starting Point
    public static Vector2d p2 = new Vector2d(-15,-63.5); //
    public static Vector2d p3 = new Vector2d(-14,-24.5); // Right of the Tall Middle Junction
    public static Vector2d p4 = new Vector2d(-9.5, -24.5); // Tall Junction Drop Location
    public static Vector2d p5 = new Vector2d(-16,-23.75); //
    public static Vector2d p6 = new Vector2d(-16,  coneStack.getY() ); //
    public static long stackDelay = 500;

    public static double stackh = 5;
    public static double stackinc = 1.25;


    double startDir = Math.toRadians(90);


    Pose2d startPose = new Pose2d(p1.getX(), p1.getY(), Math.toRadians(90));


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(p2.getX(), p2.getY()))
                .lineToSplineHeading(new Pose2d(p3.getX(), p3.getY(), Math.toRadians(0)))
                .build();

        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(p4.getX(), p4.getY()),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // was traj15.end before
        TrajectorySequence traj175 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(p5.getX(), p5.getY()))
                .lineToLinearHeading(new Pose2d(p6.getX(), coneStack.getY(), Math.toRadians(180)))
                .build();

        TrajectorySequence setupCycle = drive.trajectorySequenceBuilder(traj175.end())
                .lineToConstantHeading(new Vector2d(coneStack.getX(), coneStack.getY()))
                .build();

        TrajectorySequence toCyclePole = drive.trajectorySequenceBuilder(setupCycle.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(shortPole.getX(), coneStack.getY(), Math.toRadians(270)))
                .build();

        TrajectorySequence front = drive.trajectorySequenceBuilder(toCyclePole.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), shortPole.getY()))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(front.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), coneStack.getY()))
                .build();

        TrajectorySequence toStack = drive.trajectorySequenceBuilder(back.end())
                .lineToLinearHeading(new Pose2d(coneStack.getX() + 4, coneStack.getY(), Math.toRadians(180)))
                .forward(4)
                .build();

        TrajectorySequence center = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(-36, coneStack.getY()))
                .build();

        TrajectorySequence left = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(-59, coneStack.getY()))
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(back.end())
                .lineToConstantHeading(new Vector2d(-12, coneStack.getY()))
                .build();

        BeepArm.init(hardwareMap, this);
        drive.setPoseEstimate(startPose);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Color myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Color(RegionCenterX, RegionCenterY, RegionWidth, RegionHeight));

        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN); // Was Upright

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera Opening Error !!!!!");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addData("Push Camera Stream and tap screen to update image \nAlign the square to the cone \n \n Wait at least 5 Sec before Pressing Start", "");
        telemetry.update();
        waitForStart();


        // Get the latest frame analyzed result
        ParkingPos = myPipeline.getAnalysis();

        //Close Camera Device
        webcam.closeCameraDevice();

        // Start Pipeline for Automatic Placement
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // OpenCV webcam
      //  int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Contour_Pole myPipelinePole;
        webcam.setPipeline(myPipelinePole = new BBBDetector_Contour_Pole(CAMERA_WIDTH, CAMERA_HEIGHT,clawCenter,pixelMargin ));

        //Webcam streaming on the dashboard
        FtcDashboard.getInstance().startCameraStream(webcam, 0);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN); // Was Upright
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Camera Opening Error !!!!!");
                telemetry.update();
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        //Initialize clock
        clock = NanoClock.system();

        // Show on the screen which Parking Spot was detected
        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            telemetry.addLine("Autonomous A - Blue - Right");
            telemetry.update();

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            telemetry.addLine("Autonomous B - Green - Center");
            telemetry.update();

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            telemetry.addLine("Autonomous C - Yellow - Left");
            telemetry.update();
        }else{
                telemetry.addLine("DID NOT SEE.. SO we park at - Center");
                telemetry.update();
        }


        //Main Movement
        BeepArm.ClawFullClose(450); //wait 250 ms to make sure the cone is gripped well
        BeepArm.ViperSlideSetPos(2, 24, -1);
        drive.followTrajectorySequence(traj1);

      //  drive.followTrajectorySequence(traj15);
        sleep(250); // wait a little if the robot wiggle

        dropConeAt(RobotArm.getHIGHPOS(), myPipelinePole, drive, traj1);

        drive.followTrajectorySequence(traj175);
        BeepArm.ViperSlideSetPos(stackh, 36, 1); //Don't wait.. go back now

        drive.followTrajectorySequence(setupCycle);
        BeepArm.ClawFullClose(1000);
        BeepArm.ViperSlideSetPos(16, 36, stackDelay);
        drive.followTrajectorySequence(toCyclePole);

        dropConeAt(RobotArm.getLOWPOS(), myPipelinePole, drive, toCyclePole);

        BeepArm.ViperSlideSetPos(stackh - stackinc, 24, -1);
        drive.followTrajectorySequence(toStack);
        BeepArm.ClawFullClose(1000);
        BeepArm.ViperSlideSetPos(16, 36, stackDelay);
        drive.followTrajectorySequence(toCyclePole);

        dropConeAt(RobotArm.getLOWPOS(), myPipelinePole, drive, toCyclePole);

        BeepArm.ViperSlideSetPos(0, 24, 1);

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            drive.followTrajectorySequence(right);
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            drive.followTrajectorySequence(left);
        } else { // (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            drive.followTrajectorySequence(center);
        }
    }
    // Drop Cone function assumes the robot is near a junction
    // We supply the dropping height in inches
    public void dropConeAt(double dropHeight, @NonNull BBBDetector_Contour_Pole myPipeline, SampleMecanumDrive drive, TrajectorySequence lastTraj){
        currentPos = myPipeline.getPosition();

        BeepArm.ViperSlideSetPos(dropHeight-preDropH, 10, 1);

        double xStick;
        double yStick=0;
        double error = 0;
        double yerror = 0;

        currentPos = myPipeline.getPosition();
        error = clawCenter - currentPos;
        distAdjust = 0;

        currentAdjustTime = clock.seconds();
        while ( (Math.abs(error) > pixelMargin || BeepArm.ViperSlideGetPos() < (dropHeight-preDropH-.5) ) && (clock.seconds()-currentAdjustTime) < adjustTimeLimit  && opModeIsActive() ) {


            xStick = Kp * error;
            xStick = Math.max( -Math.abs(x), Math.min(xStick, Math.abs(x)));

            yStick = Kp * yerror;
            yStick = Math.max( -Math.abs(x), Math.min(yStick, Math.abs(x)));

            drive.strafe(-xStick, yStick);
            distAdjust+=-xStick;

            currentPos = myPipeline.getPosition();

            telemetry.addData("Current Image x", currentPos);
            telemetry.addData("Error", error);
            telemetry.addData("Strafe Power", -xStick);
            telemetry.addData("Current Image in Pixels", myPipeline.getWidthpix());
            telemetry.addData("Distance in inches", String.format("%.2f" , myPipeline.getWidth()) );
            telemetry.addData("Viper Height in inches",String.format("%.2f" , BeepArm.ViperSlideGetPos()) );
            telemetry.addData("Total Adjust distance", String.format("%.3f" ,distAdjust));


            if ( currentPos>1 ) {
                error = clawCenter - currentPos;
                telemetry.addData("Time doing Correction is: ", clock.seconds()-currentAdjustTime);
            }
            else{
                // Reset timer
                currentAdjustTime = clock.seconds();
                telemetry.addLine("Resetting time because not yellow read");
                telemetry.update();
                error=0;
            }

            if (Math.abs(error) < pixelMargin ){
                currentAdjustTime = clock.seconds();
                telemetry.addLine("Resetting time because within Margin");
                error=0;
            }

            telemetry.update();
        }
        drive.strafe(0,0);

        telemetry.addLine("Out of Loop");
        telemetry.update();
        // sleep(1500);
        double lastMove =  myPipeline.getWidth();


        BeepArm.ViperSlideSetPos(dropHeight,12,-1);

        TrajectorySequence drop;// = new TrajectorySequence;
        TrajectorySequence back;// = new TrajectorySequence;

        if (lastMove <6 ) {
            telemetry.addData("Moving inches", String.format("%.3f" ,lastMove));
            telemetry.update();
            drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    .forward(lastMove,
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(lastMove)
                    .build();

            sleep(250);
        }
        else{
            telemetry.addLine("Moving 6 inches");
            telemetry.update();

             drop = drive.trajectorySequenceBuilder(lastTraj.end())
                    .forward(6,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            back = drive.trajectorySequenceBuilder(drop.end())
                    .back(6)
                    .build();
            sleep(250);
        }
        drive.followTrajectorySequence(drop);
        BeepArm.ViperSlideSetPos(dropHeight-3,12,250);

        BeepArm.ClawFullOpen(250);
        drive.followTrajectorySequence(back);

    }
}