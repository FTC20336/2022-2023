package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotArm {

    public DcMotorEx Arm;
    public DcMotorEx ArmJoint;
    public CRServo Claw;

    static private double GEAR_3_RATIO = 2.89;
    static private double GEAR_4_RATIO = 3.61;
    static private double GEAR_5_RATIO = 5.23;

    static double COUNT_PER_DEGREE_ARM = 28 * GEAR_3_RATIO * GEAR_4_RATIO * GEAR_5_RATIO * (125.0 / 30.0) * (90.0 / 45.0) / 360;
    static double COUNT_PER_DEGREE_ARMJOINT = 28 * GEAR_3_RATIO * GEAR_4_RATIO * GEAR_5_RATIO / 360;

    // Local OpMode members
    HardwareMap hwMap = null;

    LinearOpMode MyOp = null;


    // Constructor - leave this blank for now
    public RobotArm() {

    }
    // Negative Value Closes the Claw
    public void ClawGrab(long timeout)
    {
        Claw.setPower(-.25);
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }
    public void ClawOpen(long timeout){

        if (timeout >= 0)
            MyOp.sleep(timeout);
    }
    public void ClawStop( long timeout){
        Claw.setPower(0);
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }


    public void ArmSetPos(double angle, double speed, long timeout){
        Arm.setTargetPosition( (int) (angle *COUNT_PER_DEGREE_ARM) );
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setVelocity(speed * COUNT_PER_DEGREE_ARM);

        if (timeout < 0 ){
            while (MyOp.opModeIsActive() && Arm.isBusy())
            {}

            MyOp.sleep(Math.abs(timeout));
        }

    }

    public void ArmJointSetPos(double angle, double speed, long timeout){
        ArmJoint.setTargetPosition( (int) (angle * COUNT_PER_DEGREE_ARMJOINT) );
        ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmJoint.setVelocity(speed * COUNT_PER_DEGREE_ARMJOINT);
        if (timeout < 0 ){
            while (MyOp.opModeIsActive() && Arm.isBusy())
            {}
        }
        else {
            MyOp.sleep(Math.abs(timeout));
        }
    }


    public void ArmGetPosition(){}

    public void ArmJointGetPosition(){}

    public void ArmResetEncoder(){}

    public void ArmJointResetEncoder(){}

    public void init(HardwareMap ahwMap, LinearOpMode MyOpin) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        MyOp = MyOpin;

        ArmJoint = hwMap.get(DcMotorEx.class, "arm joint");
        Arm = hwMap.get(DcMotorEx.class, "arm");
        Claw = hwMap.get(CRServo.class, "claw");

        // Reverse one of the drive motors.
        ArmJoint.setDirection(DcMotorEx.Direction.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}