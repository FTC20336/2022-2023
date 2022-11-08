package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotBase;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoUsingDetector", group="Tutorials")

public class AutoUsingDetector extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();

    private static final int CAMERA_WIDTH  = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    private static double RegionCenterX = CAMERA_WIDTH/2;
    private static double RegionCenterY = 360; // Distance in pixels from the top
    private static double RegionWidth = 50;
    private static double RegionHeight = 50;

    @Override
    public void runOpMode()
    {
        Beep.init(hardwareMap, this);

        // OpenCV webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);

        //OpenCV Pipeline
        BBBDetector_Color myPipeline;
        webcam.setPipeline(myPipeline = new BBBDetector_Color(RegionCenterX, RegionCenterY, RegionWidth, RegionHeight));

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
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
        telemetry.addData("Push Camera Stream and tap screen to update image.. Align the square to the cone","");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Detector Results:" , myPipeline.getAnalysis() );
            telemetry.update();


            if( myPipeline.getAnalysis() == BBBDetector_Color.ElementPosition.RIGHT ){
                AUTONOMOUS_C();
            }
            else if(myPipeline.getAnalysis() == BBBDetector_Color.ElementPosition.CENTER){
                AUTONOMOUS_B();
            }
            else {
                AUTONOMOUS_A();
            }


        }
    }
    public void AUTONOMOUS_A(){
        telemetry.addLine("Autonomous A - Yellow - Left");
        telemetry.update();

        Beep.strafe(24,-90,1, 1000);
        Beep.move(24, 1, 1000);
    }
    public void AUTONOMOUS_B(){
        telemetry.addLine("Autonomous B - Green - Center");
        telemetry.update();

        Beep.strafe(24,-90,1, 1000);
        Beep.move(48, 1, 1000);
        Beep.strafe(24,90,1, 1000);
    }
    public void AUTONOMOUS_C(){
        telemetry.addLine("Autonomous C - Blue - Right");
        telemetry.update();

        Beep.strafe(24,90,1, 1000);
        Beep.move(24, 1, 1000);
    }
}