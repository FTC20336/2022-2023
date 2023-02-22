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
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name="Left-TallPole-BLUE_Spline", group="Left")
public class AutoUsingDetector_Left_RR_Vision_Spline extends LinearOpMode {
    private OpenCvCamera webcam;
    private OpenCvCamera webcam2;

    RobotArm BeepArm = new RobotArm();

    AutoVisionAction AutoAction = new AutoVisionAction();

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
    public static double p2y = -52;

    public static double p3x = -13;
    public static double p3y = -46;

    public static double p4x = -13;
    public static double p4y = -24;

    public static double p5x = -16;
    public static double p5y = -23.75;

    public static double p6x = -24;
    public static double p6y = coneStack.getY();

    // GOing to Short Pole
    public static double p7x = -55;
    public static double p7y = -13 ;


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
        //Initialize BeepArm
        BeepArm.init(hardwareMap, this);


        //Initialize clock
        clock = NanoClock.system();

    /*
    .setTangent(Math.toRadians(10))
                                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
                                .splineToConstantHeading( new Vector2d(-12, -46), Math.toRadians(90))
                                .splineToSplineHeading( new Pose2d(-12, -24,), Math.toRadians(90))
                                .forward(6)
                                .setTangent(Math.toRadians(140))
                                .splineToSplineHeading(new Pose2d(-24,-11, Math.toRadians(180)),  Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-57,-12),Math.toRadians(180))
                                .forward(6) // PIck up Cone
                                .splineToConstantHeading(new Vector2d(-57,-12),Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-55,-13,Math.toRadians(315)), Math.toRadians(315))
                                .forward(6) //Drop COne
                                .setTangent(Math.toRadians(135))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(-57,-11,Math.toRadians(180)), Math.toRadians(180))
                                .forward(6) // Pickup COne
                                .splineToConstantHeading(new Vector2d(-57,-12),Math.toRadians(180))
                                .setTangent(Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-31,-13,Math.toRadians(315)), Math.toRadians(315))
                                .forward(6) //Drop COne
                                .setReversed(true)
                                .setTangent(Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(-30,-12, Math.toRadians(270)))
                                .setTangent(Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-12,-12), Math.toRadians(0))
                                */


        TrajectorySequence startTo1stDrop = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(10))
                .splineToConstantHeading(new Vector2d(p2x, p2y), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(p3x, p3y), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(p4x, p4y, Math.toRadians(0)), Math.toRadians(90))
                .build();
/*
        TrajectorySequence traj15 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToConstantHeading(new Vector2d(p4.getX()-5, p4.getY()-5),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
*/
        // was traj15.end before

/*
        TrajectorySequence setupCycle = drive.trajectorySequenceBuilder(traj1.end())
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(p6x,p6y, Math.toRadians(180)),  Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(coneStack.getX(), coneStack.getY()),Math.toRadians(180))
                .build();

        TrajectorySequence toCyclePole = drive.trajectorySequenceBuilder(traj1.end())
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
*/





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

        // Pre-Start Color
        // Show on the screen which Parking Spot was detected
        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            telemetry.addLine("Autonomous A - Blue - Right...Initial Reading.. May change");

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            telemetry.addLine("Autonomous B - Green - Center...Initial Reading.. May change");

        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            telemetry.addLine("Autonomous C - Yellow - Left...Initial Reading.. May change");
        }else{
            telemetry.addLine("DID NOT SEE.. SO we park at - Center...Initial Reading.. May change");
        }
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
        webcam.setPipeline(myPipelinePole = new BBBDetector_Contour_Pole_Cone(CAMERA_WIDTH, CAMERA_HEIGHT,clawCenter,pixelMargin, BBBDetector_Contour_Pole_Cone.conecolor.RED));

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

        // Initialize AutoAction.. Setting this LinearOp, Image processsing Pipeline, Mecanum Drive and BeepARm
        AutoAction.init(this,myPipelinePole,drive,BeepArm );

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
        drive.setPoseEstimate(startPose);
        BeepArm.ClawFullClose(450); //wait 450 ms to make sure the cone is gripped well
        BeepArm.ViperSlideSetPos(2, 24, -1);

        drive.followTrajectorySequence(startTo1stDrop);

        sleep(250); // wait a little if the robot wiggle

        Pose2d end =  AutoAction.dropConeAtNoPID(RobotArm.getHIGHPOS(), startTo1stDrop );

        TrajectorySequence after1stConeToStack = drive.trajectorySequenceBuilder(end)
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(p6x,p6y, Math.toRadians(180)),  Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(coneStack.getX(), coneStack.getY()),Math.toRadians(180))
                .addTemporalMarker(2, ()-> BeepArm.ViperSlideSetPos(stackh, 36, (long) -0.0000001)) //Don't wait.. go back now )
                .build();

        drive.followTrajectorySequence(after1stConeToStack);

        end  = AutoAction.pickConeAtNoPID(stackh, after1stConeToStack);

        drive.followTrajectorySequence(after1stConeToStack);
        TrajectorySequence after1stPickToSPole = drive.trajectorySequenceBuilder(end)
                                    .setReversed(true)
                                    .splineToConstantHeading(new Vector2d(coneStack.getX(), coneStack.getY()),Math.toRadians(180))
                                    .setTangent(Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(p7x,p7y,Math.toRadians(315)), Math.toRadians(315))
                                    .build();

        end = AutoAction.dropConeAtNoPID(RobotArm.getLOWPOS(), after1stPickToSPole);


        BeepArm.ViperSlideSetPos(0, 24, 1);


        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
                TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                .lineToConstantHeading(new Vector2d(-12, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
                TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                .lineToConstantHeading(new Vector2d(-59, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);

        } else { // (ParkingPos == BBBDetector_Color.ElementPosition.CENTER)
            TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                .lineToConstantHeading(new Vector2d(-36, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);
        }


    }

}