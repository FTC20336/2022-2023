package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.config.Config;
@Disabled
@Config
@Autonomous(name="AutoMoveTest", group="Test")

public class AutomoveTest extends LinearOpMode {
    private OpenCvCamera webcam;

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();

    @Override
    public void runOpMode() {
        Beep.init(hardwareMap, this);

        waitForStart();

        Beep.move(20, 24, 0);

    }
}
