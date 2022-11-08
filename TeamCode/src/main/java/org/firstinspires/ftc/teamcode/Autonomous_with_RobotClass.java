package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RobotBase;

@Autonomous(name="Autonomous with RobotClass", group="Auto Test")
//@Disabled
public class Autonomous_with_RobotClass extends LinearOpMode {

    //Create New Robot based on RobotBase
    RobotBase Beep = new RobotBase();


    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //Initialize Robot Class
        Beep.init(hardwareMap, this);
        //Beep.BeepArm.init(hardwareMap);

        waitForStart();


        if (opModeIsActive()) {

            // Grab Initial block
            //Beep.BeepArm.ClawGrab(1200);


            // Beep.BeepArm.ArmSetPos(40,60, -1000);
            //Beep.BeepArm.ArmJointSetPos(50,45,0);

           // Beep.move(72,24,-1);
            //Position 1
            //Beep.strafe(24, 45, 12,-6000);
            Beep.rotate(720,90,0);
            //  Beep.move(11.5,20,-1);

            //Drop the block on the top level
            //   Beep.BeepArm.ClawOpen(200);
            //   Beep.BeepArm.ClawStop(0);
/*

            // Move armjoint up a little to clear anything in the way
            Beep.BeepArm.ArmJointSetPos(60,45,-1);

            // Move back to Warehouse from Dropping the Duck
            Beep.move(-6, 20, -1);
            Beep.rotate(90, 45, -1); //sleep(2000);
            Beep.move(60,999,-1);

            // To Roll a Duck.. Uncomment this

            Beep.move(-4, 20, -1);
            Beep.rotate(180, 45, -1);
            //Start from Po1
            Beep.strafe (65,-90,20,-1);
            Beep.strafe (2,-90,10,-1);
            Beep.move(10, 5, -1);
            Beep.DuckWheelBlue();
            sleep(2500);
            Beep.DuckWheelStop();



            // To part in the small parking.. Uncomment this and comment next block
            Beep.move(-18,20,-1);
            Beep.BeepArm.ArmSetPos(70,60,-1);

            // Go to Warehouse
            Beep.strafe (20,90,15,-1);
            Beep.rotate(-105, 45, -1); //sleep(2000);
            Beep.move(100,100,-1);


            // Set Arm and ArmJoint Position to Known position so TeleOp Mode can use Encoders correctly
            Beep.BeepArm.ArmJointSetPos(0,60,500);
            Beep.BeepArm.ArmSetPos(0,60,-1);

            */
        }
    }
}

