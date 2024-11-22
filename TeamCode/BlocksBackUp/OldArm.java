package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class OldArm extends LinearOpMode{
  
  private DcMotor E1;
  private DcMotor E2;
  private DcMotor E3;
  private CRServo S2;
  
  private void oldArm(){
    if (gamepad2.left_stick_y != 0) {
      E2_POWER = gamepad2.left_stick_y * 0.5;
      if (E2_POWER < 0) {
        E2_POWER = E2_POWER / 0.5;
      }
      E2.setPower(E2_POWER);
    } else if (gamepad2.right_stick_y != 0) {
      E1_POWER = gamepad2.right_stick_y * 0.75;
      if (E1_POWER > 0) {
        E1_POWER /= 0.75;
      }
      E1.setPower(E1_POWER);
    } else if (gamepad2.dpad_up == true) {
      E3.setPower(0.5);
    } else if (gamepad2.dpad_down == true) {
      E3.setPower(-0.5);
    } else if (gamepad2.left_trigger > 0) {
      S2.setPower(1);
    } else if (gamepad2.right_trigger > 0) {
      S2.setPower(-1);
    } else {
      E1.setPower(0);
      E2.setPower(0);
      E3.setPower(0);
      E1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      E2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      E3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      S2.setPower(0);
    }
  }
  
  public void runOpMode() throws InterruptedException {
    double E1_POWER;
    double E2_POWER;
    
    E1 = hardwareMap.get(DcMotor.class, "E1");
    E2 = hardwareMap.get(DcMotor.class, "E2");
    E3 = hardwareMap.get(DcMotor.class, "E3");
    S2 = hardwareMap.get(CRServo.class, "S2");
    
    E1.setDirection(DcMotorSimple.Direction.REVERSE);
    E2.setDirection(DcMotorSimple.Direction.REVERSE);
    
    while (opModeIsActive()) {
      oldArm();
      telemetry.update();
    }
  }
  
    // todo: write your code here
}
