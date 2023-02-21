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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name="TESTDIBT USE", group="Left")
public class AutoUsingDetector_Left_RR_Vision_TESTDONTUSE extends LinearOpMode {
    private OpenCvCamera webcam;
    private OpenCvCamera webcam2;

    RobotArm BeepArm= new RobotArm();

    AutoVisionAction AutoAction;

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
    public static double Kp = 0.00035;
    public static int clawCenter = 690;
    public static double x = .25; // Max power during Strafe
    public static int pixelMargin = 35;
    private static int currentPos;

    private  NanoClock clock;
    private double currentAdjustTime;
    public static double adjustTimeLimit=1;
    private double distAdjust=0;

    //height before a drop
    public static double preDropH = 7;
    public static double preScanViperTimeout = -1;

    private BBBDetector_Color.ElementPosition ParkingPos;

    public static Vector2d coneStack = new Vector2d(-62.75+6, -13.5);
    public static Vector2d shortPole = new Vector2d(-49,-19);

    public static double p1x = -40;
    public static double p1y = -65;

    public static double p2x = -14;
    public static double p2y = -57;

    public static double p3x = -15;
    public static double p3y = -25.5;

    public static double p4x = -9.5;
    public static double p4y = -24.5;

    public static double p5x = -16;
    public static double p5y = -23.75;

    public static double p6x = -16;
    public static double p6y = coneStack.getY();

    private static Vector2d p1 = new Vector2d(p1x, p1y); // Starting Point
    private static Vector2d p2 = new Vector2d(p2x, p2y); //
    private static Vector2d p3 = new Vector2d(p3x, p3y); // Right of the Tall Middle Junction
    private static Vector2d p4 = new Vector2d(p4x, p4y); // Tall Junction Drop Location
    private static Vector2d p5 = new Vector2d(p5x, p5y); //
    private static Vector2d p6 = new Vector2d(p6x, p6y); //
    public static long stackDelay = 500;

    public static double stackh = 5;
    public static double stackinc = 1.25;

    public static double preScanDelay = 500;

    double startDir = Math.toRadians(90);


    Pose2d startPose = new Pose2d(p1.getX(), p1.getY(), Math.toRadians(90));


    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(p2x, p2y))
                .lineToConstantHeading(new Vector2d(p2x, p2y+5))
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
                .lineToLinearHeading(new Pose2d(shortPole.getX(), coneStack.getY() + 2, Math.toRadians(270)))
                .build();

        TrajectorySequence front = drive.trajectorySequenceBuilder(toCyclePole.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), shortPole.getY()))
                .build();

        TrajectorySequence back = drive.trajectorySequenceBuilder(front.end())
                .lineToConstantHeading(new Vector2d(shortPole.getX(), coneStack.getY() + 2))
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



        drive.setPoseEstimate(startPose);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline for Sleeve Color Detection
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
        BBBDetector_Contour_Pole_Cone myPipelinePole;
        webcam.setPipeline(myPipelinePole = new BBBDetector_Contour_Pole_Cone(CAMERA_WIDTH, CAMERA_HEIGHT,clawCenter,pixelMargin, BBBDetector_Contour_Pole_Cone.conecolor.BLUE));

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


        //Initialize BeepArm
        BeepArm.init(hardwareMap, this);

        // Initialize AutoAction.. Setting this LinearOp, Image processsing Pipeline, Mecanum Drive and BeepARm
        AutoAction.init(this,myPipelinePole,drive,BeepArm );

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

        // Test Code to check Color
/*
        sleep(4000);
        BeepArm.ViperSlideSetPos(stackh,35,-1);
        while (   opModeIsActive() ) {
            telemetry.addData("Current Image in Pixels", myPipelinePole.getConeDistancePixel());
            telemetry.addData("Distance in inches", String.format("%.2f", myPipelinePole.getConeDistanceInches()));
            telemetry.addData("Viper Height in inches", String.format("%.2f", BeepArm.ViperSlideGetPos()));
            telemetry.addData("Total Adjust distance", String.format("%.3f", distAdjust));

        }
*/
            //Main Movement
        BeepArm.ClawFullClose(450); //wait 250 ms to make sure the cone is gripped well
        BeepArm.ViperSlideSetPos(2, 24, -1);
        drive.followTrajectorySequence(traj1);

        sleep(250); // wait a little if the robot wiggle

        AutoAction.dropConeAt(RobotArm.getHIGHPOS(), traj1);

        drive.followTrajectorySequence(traj175);
        BeepArm.ViperSlideSetPos(stackh, 36, 1); //Don't wait.. go back now

        drive.followTrajectorySequence(setupCycle);

        AutoAction.pickConeAt(stackh, setupCycle);

        BeepArm.ViperSlideSetPos(RobotArm.getLOWPOS()-preDropH, 36, 1); //Don't wait.. go back now
        drive.followTrajectorySequence(toCyclePole);

        AutoAction.dropConeAt(RobotArm.getLOWPOS(),  toCyclePole);

        BeepArm.ViperSlideSetPos(0, 24, 1);

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            drive.followTrajectorySequence(right);
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            drive.followTrajectorySequence(left);
        } else { // (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            drive.followTrajectorySequence(center);
        }

    }

}