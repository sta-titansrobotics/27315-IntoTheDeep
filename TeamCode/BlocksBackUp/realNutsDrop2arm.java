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
public class realNutsDrop extends LinearOpMode {
    
    private DcMotor E1;
    private DcMotor E2;
    private CRServo S2;
    private boolean slowArm = false;
    private boolean b2 = false;
    private boolean a2 = false;
    private double driveCoefficient;
    private double armCoefficient;
    
    // telemetry up here
    private void telem(){
      telemetry.addLine("Gamepad1 Joystick");
      telemetry.addData("Left Stick X", gamepad1.left_stick_x);
      telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
      telemetry.addData("Right Stick X", gamepad1.right_stick_x);
      telemetry.addLine("Gamepad2 Joystick");
      telemetry.addData("Left Stick Y", gamepad2.left_stick_y);
      telemetry.addData("Right Stick Y", gamepad2.right_stick_y);
      telemetry.addLine("Arm Power");
      telemetry.addData("E1 2nd Arm", E1.getPower());
      telemetry.addData("E2 Main Arm", E2.getPower());
      telemetry.addData("rstick y2", gamepad2.right_stick_y);
      telemetry.addData("lstick y2", gamepad2.left_stick_y);
      telemetry.addData("slow arm", slowArm);
      telemetry.addData("b", gamepad2.b);
      telemetry.addData("e1bus",E1.isBusy());
      telemetry.addData("e2bus",E2.isBusy());
      telemetry.addData("E1 Position", E1.getCurrentPosition());
      telemetry.addData("E2 Position", E2.getCurrentPosition());
    }
    
    // Arm Movement
    private void arm(){
      if (slowArm){
        armCoefficient = 0.62;
      } else {
        armCoefficient = 1.;
      } 
      if (gamepad2.left_trigger > 0) {
        S2.setPower(-1);
      } else if (gamepad2.right_trigger > 0) {
        S2.setPower(1);
      } else {
        S2.setPower(0);
      }
      if ((gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y)){
        if (gamepad2.x){
          E2.setTargetPosition(12);
          E1.setTargetPosition(654);
        } else if (gamepad2.a){
          E2.setTargetPosition(12);
          E1.setTargetPosition(654);
        } else{
          E2.setTargetPosition(3362);
          E1.setTargetPosition(769);
        }
        E2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        E1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (E2.getTargetPosition() > E2.getCurrentPosition()){
            E2.setPower(.8);
        } else if(E2.getTargetPosition()< E2.getCurrentPosition()){
            E2.setPower(-.8); 
        }else{
            E2.setPower(0);
        }
        if (E1.getTargetPosition() > E1.getCurrentPosition()){
            E1.setPower(0.5);
        } else if (E1.getTargetPosition() < E1.getCurrentPosition()){
            E1.setPower(-0.5);
        }else{
            E1.setPower(0);
        }
      }
      if (!(E2.isBusy()||E1.isBusy())){
        E2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        E1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        E2.setPower(-gamepad2.left_stick_y * armCoefficient * armCoefficient);
        E1.setPower(-gamepad2.right_stick_y * armCoefficient);
      }
    }
  
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("4");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("3");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("1");

        E1 = hardwareMap.get(DcMotor.class, "E1");
        E2 = hardwareMap.get(DcMotor.class, "E2");
        S2 = hardwareMap.get(CRServo.class, "S2");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        E1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        E2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        E2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        E1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            
            // Drivetrain Movement code - Field Centric
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (true) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            //Gamepad2 Arm Movement----
            if (!gamepad2.back && a2){
              slowArm = !slowArm;
            }
            arm();
            //telemetry code
            telemetry.addLine("Drivetrain Motor Power");
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telem();
            telemetry.update();
        }
    }
}
