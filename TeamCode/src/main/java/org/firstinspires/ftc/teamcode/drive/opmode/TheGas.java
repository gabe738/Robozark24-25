package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


@TeleOp(name = "TheGas")
public class TheGas extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
//    private CRServo extend;
//    private DcMotorEx angle;
//    private CRServo wheel;
    private double wheelPow = 0;
    private double angleAtBottom;
    long startTime;
    double startSpeed = 0.5;  // Speed of the CRServo
    long timeMoving = 0;      // Time CRServo has been moving
    boolean isMoving = false;

    int dir;
    double distance = 0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

//        extend = hardwareMap.get(CRServo.class, "extend");
//        angle = hardwareMap.get(DcMotorEx.class, "angle");
//        wheel = hardwareMap.get(CRServo.class, "wheel");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

//        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer();

//        angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        angleAtBottom = angle.getCurrentPosition();
//        while (angle.getCurrentPosition() > angleAtBottom + 100){
//            angle.setPower(-0.5);
//        }
//        angle.setPower(0);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftY = -gamepad1.left_stick_y; // Forward/Backward
            double leftX = gamepad1.left_stick_x;  // Left/Right
            double rightX = gamepad1.right_stick_x; // Rotate

            // O = gamepad2 = arm movement
//            double controlLength = gamepad2.left_stick_y; // extend arm
//            double controlAngle = gamepad2.right_stick_y; // lower/raise arm
//            boolean spit = gamepad2.dpad_up; // drop game objects
//            boolean spitRoll = gamepad2.right_bumper; // increase spit
//            boolean suck = gamepad2.dpad_down; // pick up objects
//            boolean suckRoll = gamepad2.left_bumper; // increase suck

            // Calculate motor power
            double frontLeftPower = leftY - leftX + rightX;
            double rearLeftPower = leftY + leftX + rightX;
            double frontRightPower = leftY + leftX - rightX;
            double rearRightPower = leftY - leftX - rightX;

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

            // arm power

//            if (controlLength > 0.5){
//                startExtend(1);
//                dir = 1;
//            } else if (controlLength < -0.5){
//                startExtend(-1);
//                dir = -1;
//            } else if (controlLength < 0.5 && controlLength > -0.5){
//                endExtend();
//            }
////            extend.setPower(controlLength);
//            distance += estimateMovement() * dir;
//            angle.setPower(controlAngle*0.2);

//            if (spit && spitRoll){
//                wheelPow += 0.05;
//            } else if (spit) {
//                wheelPow = 0.5;
//            } else if (suck && suckRoll) {
//                wheelPow += -0.05;
//            } else if (suck){
//                wheelPow = -0.5;
//            } else {
//                wheelPow = 0;
//            }
//            wheel.setPower(wheelPow);
//            if (spit) {
//                wheel.setPower(0.5);
//            } else if (suck){
//                wheel.setPower(-0.5);
//            } else {
//                wheel.setPower(0);
//            }
            
            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
//            telemetry.addData("Extend Power", controlLength);
//            telemetry.addData("Angle Power", controlAngle);
//            telemetry.addData("Is sucking", suck);
//            telemetry.addData("Is spitting", spit);
            telemetry.update();
        }
    }
//    private void startExtend(double dir){
//        extend.setPower(dir);
//        startTime = System.currentTimeMillis();
//        isMoving = true;
//    }
//
//    private void endExtend(){
//        extend.setPower(0);
//        timeMoving = System.currentTimeMillis() - startTime;
//        isMoving = false;
//    }
//
//    public double estimateMovement() {
//        // Example: speed in rotations per second (adjust based on your CRServo's speed characteristics)
//        double rotationSpeed = 1.0;  // 1 rotation per second (as an example)
//        if (isMoving) {
//            return rotationSpeed * (System.currentTimeMillis() - startTime) / 1000.0;  // Rotations
//        } else {
//            return rotationSpeed * timeMoving / 1000.0;
//        }
//    }
}
