package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.roadrunner_files.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="AutoUsingDetector_Left", group="Tutorials")

public class AutoUsingDetector_Left extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();



    // Dont really need to change this.
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Change these values to move the little square/region where we check what color we see
    private static double RegionCenterX = 875; // Distance in pixels from the Left
    private static double RegionCenterY = 340; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;

    private BBBDetector_Color.ElementPosition ParkingPos;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Pose2d startPose = new Pose2d(-40, -65, Math.toRadians(90));

    TrajectorySequence toPole  = drive.trajectorySequenceBuilder(startPose)
            .setTangent(Math.toRadians(10))
            .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(90))
            .lineToConstantHeading( new Vector2d(-12, -40))
            .splineToConstantHeading(new Vector2d(2, -32), Math.toRadians(90))
            .lineToConstantHeading(new Vector2d(2, -30))
            .build();

    TrajectorySequence setupCycle  = drive.trajectorySequenceBuilder(toPole.end())
            .lineToConstantHeading(new Vector2d(2, -36))
            .lineToConstantHeading(new Vector2d(-12, -36))
            .lineToLinearHeading(new Pose2d(-12, -13, Math.toRadians(180)))
            .lineToConstantHeading(new Vector2d(-60, -12))
            .build();

    TrajectorySequence toCyclePole = drive.trajectorySequenceBuilder(setupCycle.end())
            .lineToSplineHeading(new Pose2d(-19, -7.5, Math.toRadians(90)))
            .build();

    TrajectorySequence resetCycle = drive.trajectorySequenceBuilder(toCyclePole.end())
            .lineToSplineHeading(new Pose2d(-60, -13, Math.toRadians(180)))
            .build();

    @Override
    public void runOpMode() {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(startPose);

        Beep.init(hardwareMap, this);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Color myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Color(RegionCenterX, RegionCenterY, RegionWidth, RegionHeight));

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            AUTONOMOUS_A();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            AUTONOMOUS_B();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            AUTONOMOUS_C();
        }


    }

    public void DropCone(){
        Beep.BeepArm.ClawFullClose(750);
        Beep.BeepArm.ViperSlideSetPos(2,20, 1000);
        Beep.strafe(28, 80, 18, 0);
        Beep.move(24, 18, 0);
        // Viper up but don't wait while strafing
        Beep.BeepArm.ViperSlideSetPos(34,20, 0);
        Beep.strafe(12, 90, 18, 0);
        Beep.move(4.5, 8, 0);
        Beep.BeepArm.ViperSlideSetPos(30, 6, 750);
        Beep.BeepArm.ClawFullOpen(750);
        Beep.move(-4, 24, 0);
    }

    public void runAuto(int cycles){
        Beep.BeepArm.ClawFullClose(750);
        Beep.BeepArm.ViperSlideSetPos(34, 6, 0);
        drive.followTrajectorySequence(toPole);
        Beep.BeepArm.ClawFullOpen(750);
        Beep.BeepArm.ViperSlideSetPos(5, 6, 0);
        drive.followTrajectorySequence(setupCycle);
        for (int i = 0; i <= cycles; i++) {
            if (5 - (1 * i) < 0) {
                break;
            }
            Beep.BeepArm.ClawFullClose(750);
            Beep.BeepArm.ViperSlideSetPos(34, 6, 0);
            drive.followTrajectorySequence(toCyclePole);
            Beep.BeepArm.ClawFullOpen(500);
            Beep.BeepArm.ViperSlideSetPos((5 - (1 * i)), 6, 0);
            drive.followTrajectorySequence(resetCycle);
        }
    }



    public void AUTONOMOUS_A() {
        telemetry.addLine("Autonomous A - Blue - Right");
        telemetry.update();
        //DropCone();

        //Beep.strafe(14, -90, 12, 0);
        //Beep.strafe(56, 180, 20, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 20, 4000);
    }

    public void AUTONOMOUS_B() {
        telemetry.addLine("Autonomous B - Green - Center");
        telemetry.update();
        /*

        DropCone();
        Beep.strafe(16, -90, 12, 0);
        Beep.move(21, 18, 0);
        Beep.strafe(23, -90, 12, 0);

         */
       // Beep.strafe(36, 180, 20, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 2000, 4000);
    }

    public void AUTONOMOUS_C() {
        telemetry.addLine("Autonomous C - Yellow - Left");
        telemetry.update();
        /*
        DropCone();
        Beep.strafe(16, -90, 12, 0);
        Beep.move(21, 18, 0);
        Beep.strafe(46, -90, 12, 0);

        */
        //Beep.strafe(12, 180, 20, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 20, 4000);

    }
}