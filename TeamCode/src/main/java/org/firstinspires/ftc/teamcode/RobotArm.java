package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotArm {


    public DcMotorEx ViperSlideMotor;
    public Servo Claw;
    static private double CLAW_FULL_OPEN_POS = 0.194;
    static private double SWING_ARM_RATIO = 188;
    static private double VIPER_SLIDE_RATIO = 19.2;
    private static double HIGHPOS = 32.5;
    private static double MIDDLEPOS  = 24;
    private static double LOWPOS = 14;

    static double COUNT_PER_DEGREE_ARM = 28 * SWING_ARM_RATIO / 360;
    static double COUNT_PER_DEGREE_SLIDE = 28 * VIPER_SLIDE_RATIO / 360;

    static double SLIDE_TURN_PER_INCH = (38.4 / 8.7) * 28;

    // Local OpMode members
    HardwareMap hwMap = null;
    LinearOpMode MyOp = null;

    public static double getHIGHPOS() {
        return HIGHPOS;
    }

    public static double getMIDDLEPOS() {
        return MIDDLEPOS;
    }

    public static double getLOWPOS() {
        return LOWPOS;
    }


    // Constructor - leave this blank for now
    public RobotArm() {

    }
    // Negative Value Closes the Claw
    public void ClawGrab(long timeout, int repeat)
    {
        if (repeat == 0)
            repeat = 1;
        Claw.setPosition(Claw.getPosition() + (0.002 * repeat));
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }
    public void ClawOpen(long timeout, int repeat)
    {
        if (repeat == 0)
            repeat = 1;
        Claw.setPosition(Claw.getPosition() - (0.002 * repeat));
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }
    public void ClawFullOpen(long timeout)
    {
        Claw.setPosition(0);
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }
    public void ClawFullClose(long timeout)
    {
        Claw.setPosition(CLAW_FULL_OPEN_POS);
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }

    public void ClawToPos(double position, long timeout)
    {
        if (position > 0 || position < CLAW_FULL_OPEN_POS) {
            Claw.setPosition(position);
        }
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }

    public void ClawStop( long timeout)
            {
        Claw.setPosition(Claw.getPosition());
        if (timeout >= 0)
            MyOp.sleep(timeout);
    }

/*
    public void SwingyArmSetPos(double angle, double speed, long timeout){
        SwingyArmMotor.setTargetPosition( (int) (angle *COUNT_PER_DEGREE_ARM) );
        SwingyArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SwingyArmMotor.setVelocity(speed * COUNT_PER_DEGREE_ARM);

        if (timeout < 0 ){
            while (MyOp.opModeIsActive() && SwingyArmMotor.isBusy())
            {}

            MyOp.sleep(Math.abs(timeout));
        }

    }
*/

    public void ViperSlideSetPos(double length, double speed, long timeout){
        ViperSlideMotor.setTargetPosition( (int) (length * SLIDE_TURN_PER_INCH) );
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ViperSlideMotor.setVelocity(speed * SLIDE_TURN_PER_INCH);
        if (timeout < 0 ){
            while (MyOp.opModeIsActive() && ViperSlideMotor.isBusy())
            {}
        }
        else {
            MyOp.sleep(Math.abs(timeout));
        }
    }

    public double ViperSlideGetPos(){
        return (ViperSlideMotor.getCurrentPosition() /  SLIDE_TURN_PER_INCH);
    }




    public void init(HardwareMap ahwMap, LinearOpMode MyOpin) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        MyOp = MyOpin;

        ViperSlideMotor = hwMap.get(DcMotorEx.class, "ViperSlideMotor");
       // SwingyArmMotor = hwMap.get(DcMotorEx.class, "SwingyArmMotor");
        Claw = hwMap.get(Servo.class, "claw");

        // Reverse one of the drive motors.
        ViperSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        //SwingyArmMotor.setDirection(DcMotor.Direction.REVERSE);

      //  SwingyArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // SwingyArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ViperSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       // SwingyArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ViperSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}