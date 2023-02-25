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
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequenceBuilder;
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
    public static double RegionCenterX = 855; // Distance in pixels from the Left
    public static double RegionCenterY = 340; // Distance in pixels from the top
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

    private BBBDetector_Color.ElementPosition ParkingPos;

    public static double poleApproachDist = 15; // Pole Center to Center of Robot
    public static double coneApproachDist = 19; // Cone CEnter to CEnter of Robot

    public static Vector2d coneStack = new Vector2d(-71, -9.5); // Field is not really 6'..
    public static Vector2d shortPole = new Vector2d(-51,-24);
    public static Vector2d mediumPole = new Vector2d(-26,-24);

    public static long stackDelay = 500;

    public static double stackh = 4.75;
    public static double stackinc = 1.25;


    public static double p1x = -39.25; //
    public static double p1y = -63.75; // 70.75" - 1/2 Robot Length (14")

    public static double p2x = -15; //24.25
    public static double p2y = -52; // 11.75

    public static double p3x = -15;
    public static double p3y = -46;

    public static double p4x = -poleApproachDist;
    public static double p4y = -22;

    public static double p5x = -16;
    public static double p5y = -23.75;

    public static double p6x = -24;
    public static double p6y = coneStack.getY();

    public static double p7yOffset = 3;

    // GOing to Short Pole
    public static double p7x;
    public static double p7y;


    private static Vector2d p1 = new Vector2d(p1x, p1y); // Starting Point
    private static Vector2d p2 = new Vector2d(p2x, p2y); //
    private static Vector2d p3 = new Vector2d(p3x, p3y); // Right of the Tall Middle Junction
    private static Vector2d p4 = new Vector2d(p4x, p4y); // Tall Junction Drop Location
    private static Vector2d p5 = new Vector2d(p5x, p5y); //
    private static Vector2d p6 = new Vector2d(p6x, p6y); //

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

        // Drive to Tall pool for a Drop
        drive.followTrajectorySequence(startTo1stDrop);

        Pose2d end =  AutoAction.dropConeAtNoPID(RobotArm.getHIGHPOS(), startTo1stDrop );

       // drive.setPoseEstimate(new Pose2d(-6,-24,Math.toRadians(0)));

        // Create Trajectory to go to the Cone Stack.. Stop short to Camera REad
        TrajectorySequence after1stConeToStack = drive.trajectorySequenceBuilder(end)
                .setTangent(Math.toRadians(140))
                .splineToSplineHeading(new Pose2d(p6x,p6y, Math.toRadians(180)),  Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL *1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(coneStack.getX()+coneApproachDist, coneStack.getY()),Math.toRadians(180))
                .addTemporalMarker(2, ()-> BeepArm.ViperSlideSetPos(stackh, 36, (long) 0)) //Don't wait.. go back now )
                .build();

        // Drive to the Cone Stack
        drive.followTrajectorySequence(after1stConeToStack);


        // Pickup First Cone
        end  = AutoAction.pickConeAtNoPID(stackh, after1stConeToStack);


        // Create Trajectory to drop on Short Pole
        double dropAngle = 270;/*
        p7x = shortPole.getX() - coneApproachDist * Math.cos(Math.toRadians(dropAngle));
        p7y = shortPole.getY() - coneApproachDist * Math.sin(Math.toRadians(dropAngle));
        TrajectorySequence drop2 = drive.trajectorySequenceBuilder(end)
                                    .setTangent(Math.toRadians(0))
                                    //.splineToSplineHeading(new Pose2d(end.getX()+5, end.getY()+2,Math.toRadians(225)),Math.toRadians(0))
                                   // .setTangent(Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(p7x,p7y,Math.toRadians(dropAngle)), Math.toRadians(0))
                                    .build();

        // Drive to Short Pole
        drive.followTrajectorySequence(drop2);

        // Drop to Short Pole
        end = AutoAction.dropConeAtNoPID(RobotArm.getLOWPOS(), drop2);

         */

      //  drive.setPoseEstimate(shortPole.getX()-6.75, shortPole.getY(), Math.toRadians(0)))=;
/*
        // Create Trajectory to drive to Cone Stack
        TrajectorySequence backToStack = drive.trajectorySequenceBuilder(end)
                .setTangent(Math.toRadians(dropAngle))
               // .setReversed(true)
                .splineToSplineHeading(new Pose2d(coneStack.getX()+coneApproachDist, coneStack.getY(),Math.toRadians(180)), Math.toRadians(180))
                .build();

        drive.followTrajectorySequence(backToStack);

        // Pickup Second Cone
        end  = AutoAction.pickConeAtNoPID(stackh-stackinc, backToStack);
        /*
 */


        // Create Trajectory to drop on medium Pole
        dropAngle = 270;
        p7x = mediumPole.getX() - coneApproachDist * Math.cos(Math.toRadians(dropAngle));
        p7y = mediumPole.getY() - coneApproachDist * Math.sin(Math.toRadians(dropAngle)) - p7yOffset;
        TrajectorySequence drop3 = drive.trajectorySequenceBuilder(end)
                .lineToConstantHeading(new Vector2d(end.getX()+20, end.getY()))
               // .setTangent(Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(p7x,p7y,Math.toRadians(dropAngle)))
                .build();

        // Drive to Medium Pole
        drive.followTrajectorySequence(drop3);

        // Drop to Medium Pole
        end = AutoAction.dropConeAtNoPID(RobotArm.getMIDDLEPOS(), drop3);

        BeepArm.ViperSlideSetPos(0, 24, 0);


        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
                TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                        //.setReversed(true)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading( new Vector2d(end.getX(), end.getY()+2),Math.toRadians(270))
                        .lineToConstantHeading(new Vector2d(-14, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
                TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading( new Vector2d(end.getX(), end.getY()+2),Math.toRadians(270))
                        .lineToConstantHeading(new Vector2d(-60, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);

        } else { // (ParkingPos == BBBDetector_Color.ElementPosition.CENTER)
            TrajectorySequence park = drive.trajectorySequenceBuilder(end)
                    .setTangent(Math.toRadians(90))
                    .splineToConstantHeading( new Vector2d(end.getX(), end.getY()+2),Math.toRadians(270))
                    .lineToConstantHeading(new Vector2d(-38, coneStack.getY()))
                .build();
            drive.followTrajectorySequence(park);
        }

    }

}