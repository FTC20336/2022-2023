package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoUsingDetector_Right", group="Right")

public class AutoUsingDetector_RightSide extends LinearOpMode {
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

        // Get the latest frame analyzed result
        ParkingPos = myPipeline.getAnalysis();

        if (ParkingPos == BBBDetector_Color.ElementPosition.RIGHT) {
            AUTONOMOUS_C();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.CENTER) {
            AUTONOMOUS_B();
        } else if (ParkingPos == BBBDetector_Color.ElementPosition.LEFT) {
            AUTONOMOUS_A();
        }

    }

    public void DropCone(){
        Beep.BeepArm.ClawFullClose(750);
        Beep.BeepArm.ViperSlideSetPos(2,20, 1000);
        Beep.strafe(22, -80, 18, 0);
        Beep.move(24, 18, 0);
        // Viper up but don't wait while strafing
        Beep.BeepArm.ViperSlideSetPos(34,20, 0);
        Beep.strafe(10.5, -90, 18, 0);
        Beep.move(6, 8, 0);
        Beep.BeepArm.ViperSlideSetPos(30, 6, 750);
        Beep.BeepArm.ClawFullOpen(750);
        Beep.move(-4, 24, 0);
    }


    public void AUTONOMOUS_A() {
        telemetry.addLine("Autonomous A - Yellow - Left");
        telemetry.update();
        DropCone();

        Beep.strafe(12, 90, 12, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 20, 4000);
    }

    public void AUTONOMOUS_B() {
        telemetry.addLine("Autonomous B - Green - Center");
        telemetry.update();

        DropCone();

        Beep.strafe(11.5, 90, 12, 0);
        Beep.move(21.5, 18, 0);
        Beep.strafe(24, 90, 12, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 20, 4000);
    }

    public void AUTONOMOUS_C() {
        telemetry.addLine("Autonomous C - Blue - Right");
        telemetry.update();

        DropCone();

        Beep.strafe(11.5, 90, 12, 0);
        Beep.move(21.5, 18, 0);
        Beep.strafe(48, 90, 12, 0);
        Beep.BeepArm.ViperSlideSetPos(0, 20, 4000);

    }
}