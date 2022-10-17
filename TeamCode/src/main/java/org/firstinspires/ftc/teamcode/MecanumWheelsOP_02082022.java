package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "MecanumWheelsOP_02082022 (Blocks to Java)")
public class MecanumWheelsOP_02082022 extends LinearOpMode {

  private DcMotor frontRightWheel;
  private DcMotor arm;
  private DcMotor frontLeftMotor;
  private DcMotor MotorLeft;
  private DcMotor MotorRight;
  private DcMotor armjoint;
  private DcMotor topwheelmotor;
  private CRServo claw;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int gotBlock_;
    int Duck;
    float yVel;
    float xVel;
    double rx;
    double denominator;

    frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
    arm = hardwareMap.get(DcMotor.class, "arm");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    MotorLeft = hardwareMap.get(DcMotor.class, "MotorLeft");
    MotorRight = hardwareMap.get(DcMotor.class, "MotorRight");
    armjoint = hardwareMap.get(DcMotor.class, "arm joint");
    topwheelmotor = hardwareMap.get(DcMotor.class, "topwheelmotor");
    claw = hardwareMap.get(CRServo.class, "claw");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      gotBlock_ = 0;
      Duck = 0;
      frontRightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
      arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      while (opModeIsActive()) {
        // Put loop blocks here.
        yVel = -gamepad1.left_stick_y;
        xVel = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x * 1.1;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(yVel), Math.abs(xVel), Math.abs(rx))), 1));
        frontLeftMotor.setPower((yVel + xVel + rx) / denominator);
        MotorLeft.setPower(((yVel - xVel) + rx) / denominator);
        frontRightWheel.setPower(((yVel - xVel) - rx) / denominator);
        MotorRight.setPower(((yVel + xVel) - rx) / denominator);
        if (gamepad1.dpad_up) {
          arm.setPower(1);
          telemetry.addData("but A press", arm.getPower());
        }
        if (gamepad1.dpad_down) {
          arm.setPower(-1);
        }
        if (!gamepad1.a && !gamepad1.b) {
          arm.setPower(0);
        }
        if (gamepad1.a) {
          armjoint.setPower(-0.5);
        }
        if (gamepad1.b) {
          armjoint.setPower(0.5);
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
          armjoint.setPower(0);
        }
        if (gamepad1.y) {
          if (Duck == 0) {
            Duck = 1;
            topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topwheelmotor.setTargetPosition(-100000);
            topwheelmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) topwheelmotor).setVelocity(2100);
          } else {
            if (topwheelmotor.getCurrentPosition() <= -1500) {
              ((DcMotorEx) topwheelmotor).setVelocity(5000);
            }
            if (topwheelmotor.getCurrentPosition() <= -4500) {
              topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
          }
        }
        if (gamepad1.x) {
          if (Duck == 0) {
            Duck = 1;
            topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            topwheelmotor.setTargetPosition(100000);
            topwheelmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) topwheelmotor).setVelocity(2150);
          } else {
            if (topwheelmotor.getCurrentPosition() >= 1450) {
              ((DcMotorEx) topwheelmotor).setVelocity(6000);
            }
            if (topwheelmotor.getCurrentPosition() >= 4500) {
              topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
          }
        }
        if (!gamepad1.x && !gamepad1.y) {
          topwheelmotor.setPower(0);
          Duck = 0;
        }
        if (gamepad1.right_bumper) {
          claw.setPower(-0.5);
        }
        if (gamepad1.left_bumper) {
          claw.setPower(0.5);
          gotBlock_ = 0;
        }
        if (gamepad1.start) {
          claw.setPower(-0.5);
          gotBlock_ = 1;
        }
        if (!gamepad1.right_bumper && !gamepad1.left_bumper && gotBlock_ != 1) {
          claw.setPower(0);
        }
        telemetry.addData("YVel", yVel);
        telemetry.addData("XVel", xVel);
        telemetry.addData("Rotation", rx);
        telemetry.update();
      }
    }
  }
}
