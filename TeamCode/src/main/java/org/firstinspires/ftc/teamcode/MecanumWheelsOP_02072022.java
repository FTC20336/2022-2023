package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
@Disabled
@TeleOp(name = "MecanumWheelsOP_02072022 (Blocks to Java)")
public class MecanumWheelsOP_02072022 extends LinearOpMode {

  private DcMotor frontRightWheel;
  private DcMotor arm;
  private DcMotor frontLeftMotor;
  private DcMotor MotorLeft;
  private DcMotor MotorRight;
  private DcMotor ArmAsDcMotor;
  private DcMotor armjoint;
  private DcMotor topwheelmotor;
  private CRServo claw;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int gotBlock_;
    float yVel;
    float xVel;
    double rx;
    double denominator;

    frontRightWheel = hardwareMap.get(DcMotor.class, "frontRightWheel");
    arm = hardwareMap.get(DcMotor.class, "arm");
    frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
    MotorLeft = hardwareMap.get(DcMotor.class, "MotorLeft");
    MotorRight = hardwareMap.get(DcMotor.class, "MotorRight");
    ArmAsDcMotor = hardwareMap.get(DcMotor.class, "ArmAsDcMotor");
    armjoint = hardwareMap.get(DcMotor.class, "arm joint");
    topwheelmotor = hardwareMap.get(DcMotor.class, "topwheelmotor");
    claw = hardwareMap.get(CRServo.class, "claw");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      gotBlock_ = 0;
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
        if (gamepad1.a) {
          ArmAsDcMotor.setPower(1);
        }
        if (gamepad1.b) {
          ArmAsDcMotor.setPower(-1);
        }
        if (!gamepad1.a && !gamepad1.b) {
          ArmAsDcMotor.setPower(0);
        }
        if (gamepad1.dpad_up) {
          armjoint.setPower(-0.5);
        }
        if (gamepad1.dpad_down) {
          armjoint.setPower(0.5);
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
          armjoint.setPower(0);
        }
        if (gamepad1.x) {
          topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          topwheelmotor.setTargetPosition(3500);
          topwheelmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) topwheelmotor).setVelocity(2000);
          while (topwheelmotor.getCurrentPosition() <= 1500) {
            telemetry.addData("topwheelvel", ((DcMotorEx) topwheelmotor).getVelocity());
            telemetry.addData("topwheelltarpos", topwheelmotor.getCurrentPosition());
            telemetry.update();
            if (false) {
            }
          }
          ((DcMotorEx) topwheelmotor).setVelocity(5000);
          while (topwheelmotor.isBusy()) {
            telemetry.addData("topwheelvel", ((DcMotorEx) topwheelmotor).getVelocity());
            telemetry.addData("topwheelltarpos", topwheelmotor.getTargetPosition());
            telemetry.update();
          }
        }
        if (gamepad1.y) {
          topwheelmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          topwheelmotor.setTargetPosition(-1500);
          topwheelmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) topwheelmotor).setVelocity(1000);
          while (topwheelmotor.isBusy()) {
            telemetry.addData("topwheelvel", ((DcMotorEx) topwheelmotor).getVelocity());
            telemetry.addData("topwheeltarpos", topwheelmotor.getTargetPosition());
            telemetry.update();
          }
          topwheelmotor.setTargetPosition(-4000);
          topwheelmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          ((DcMotorEx) topwheelmotor).setVelocity(2000);
          while (topwheelmotor.isBusy()) {
            telemetry.addData("topwheelvel", ((DcMotorEx) topwheelmotor).getVelocity());
            telemetry.addData("topwheeltarpos", topwheelmotor.getTargetPosition());
            telemetry.update();
          }
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
