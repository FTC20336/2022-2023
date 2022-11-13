package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotBase;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="AutoUsingDetector", group="Tutorials")

public class AutoUsingDetector extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static double RegionCenterX = 850;
    private static double RegionCenterY = 360; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;

    private BBBDetector_Color.ElementPosition ParkingPos;

    @Override
    public void runOpMode() {
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


        ParkingPos = myPipeline.getAnalysis();

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            AUTONOMOUS_C();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            AUTONOMOUS_C();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            AUTONOMOUS_C();
        }

    }

    public void AUTONOMOUS_A() {
        telemetry.addLine("Autonomous A - Yellow - Left");
        telemetry.update();

        Beep.BeepArm.ClawGrab(0, 2);
        Beep.strafe(27, -85, 12, 0);
        Beep.move(24, 12, 0);
        Beep.BeepArm.SwingyArmSetPos(Beep.BeepArm.SwingyArmMotor.getCurrentPosition(), 0, 0);
        Beep.BeepArm.ViperSlideSetPos(Beep.BeepArm.ViperSlideMotor.getCurrentPosition(), 0, 0);
        Beep.BeepArm.ClawOpen(0, 2);
        Beep.BeepArm.ViperSlideSetPos(Beep.BeepArm.ViperSlideMotor.getCurrentPosition(), 0, 0);
    }

    public void AUTONOMOUS_B() {
        telemetry.addLine("Autonomous B - Green - Center");
        telemetry.update();

        Beep.strafe(27, -85, 12, 0);
        Beep.move(48, 12, 0);
        Beep.strafe(24, 90, 12, 0);
    }

    public void AUTONOMOUS_C() {
        telemetry.addLine("Autonomous C - Blue - Right");
        telemetry.update();

        //Beep.BeepArm.ClawGrab(0, 2);
        //Beep.strafe(26, 90, 12, 0);
        //Beep.move(24, 12, 0);
        //Beep.BeepArm.SwingyArmSetPos(90, 50, 1000);
        //Beep.BeepArm.SwingyArmSetPos(Beep.BeepArm.SwingyArmMotor.getCurrentPosition(), 20, -1);
        telemetry.addLine("Arm Swung");
        telemetry.update();
        Beep.BeepArm.ViperSlideSetPos(30,20, 1000);
        telemetry.addLine("Slide Extended");
        telemetry.update();
        //Beep.BeepArm.ClawOpen(0, 2);
        //Beep.BeepArm.ViperSlideSetPos(0, 20, 1000);
        telemetry.addLine("Slide Retracted");
        telemetry.update();

    }
}
