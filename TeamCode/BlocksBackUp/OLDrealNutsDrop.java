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
public class OLDrealNutsDrop extends LinearOpMode {
    
    private DcMotor E1;
    private DcMotor E2;
    private DcMotor E3;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("4");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("3");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("1");
        
        double E1_POWER;
        double E2_POWER;

        E1 = hardwareMap.get(DcMotor.class, "E1");
        E2 = hardwareMap.get(DcMotor.class, "E2");
        E3 = hardwareMap.get(DcMotor.class, "E3");

        // Reverse the right side motors. This may be wrong for your setup;
        // If your robot moves backwards when commanded to go forwards,;
        // reverse the left side instead.;
        // See the note about this earlier on this page.;
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        E1.setDirection(DcMotor.Direction.REVERSE);
        E2.setDirection(DcMotor.Direction.REVERSE);
        
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
            if (gamepad1.options) {
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
            if (gamepad2.left_stick_y != 0) {
              E2_POWER = gamepad2.left_stick_y * 0.5;
              if (E2_POWER < 0) {
                E2_POWER = E2_POWER / 0.5;
              }
              E2.setPower(E2_POWER);
            } else if (gamepad2.right_stick_y != 0) {
              E1_POWER = gamepad2.right_stick_y * 0.75;
              if (E1_POWER < 0) {
                E1_POWER /= 0.75;
              }
              E1.setPower(E1_POWER);
            } else if (gamepad2.dpad_up == true) {
              E3.setPower(0.5);
            } else if (gamepad2.dpad_down == true) {
              E3.setPower(-0.5);
            } else if (gamepad2.left_trigger > 0) {
            } else if (gamepad2.x == true) {
              E1.setPower(0.25);
              E3.setPower(1);
              sleep(100);
            } else if (gamepad2.b == true) {
              E2.setPower(1);
              sleep(1000);
              E2.setPower(0);
            } else if (gamepad2.a == true) {
              E1.setPower(-1);
              sleep(1000);
              E1.setPower(0);
              E3.setPower(-1);
              sleep(1000);
              E3.setPower(0);
            } else if (gamepad2.right_trigger > 0) {
            } else {
              E1.setPower(0);
              E2.setPower(0);
              E3.setPower(0);
              E1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              E2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
              E3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            //telemetry code
            telemetry.addLine("Gamepad1 Joystick");
            telemetry.addData("Left Stick X", gamepad1.left_stick_x);
            telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
            telemetry.addData("Right Stick X", gamepad1.right_stick_x);
            telemetry.addLine("Drivetrain Motor Power");
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("BL Power", backLeftPower);
            telemetry.addData("BR Power", backRightPower);
            telemetry.addLine("Gamepad2 Joystick");
            telemetry.addData("Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Right Stick Y", gamepad2.right_stick_y);
            telemetry.addLine("Arm Power");
            telemetry.addData("E1 2nd Arm", gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
