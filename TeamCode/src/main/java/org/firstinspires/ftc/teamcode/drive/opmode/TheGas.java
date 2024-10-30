package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "TheGas")
public class TheGas extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            double leftY = -gamepad1.left_stick_y; // Forward/Backward
            double leftX = gamepad1.left_stick_x;  // Left/Right
            double rightX = gamepad1.right_stick_x; // Rotate

            // Calculate motor power
            double frontLeftPower = leftY + leftX + rightX;
            double rearLeftPower = leftY - leftX + rightX;
            double frontRightPower = leftY - leftX - rightX;
            double rearRightPower = leftY + leftX - rightX;

            // Normalize the power values if any exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                rearLeftPower /= maxPower;
                frontRightPower /= maxPower;
                rearRightPower /= maxPower;
            }

            // Set motor power
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(rearLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(rearRightPower);

            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.update();
        }
    }
}
