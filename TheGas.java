package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AutoMethods;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name = "TheGas")
public class TheGas extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx extend;
    private Servo angle;
    private Servo claw;
    double anglePos = 0.48;
    double angleMult = -0.0033;
    double extendMult = -0.6;
    double clawPower = 0.0;
    double clawPos = 0.3;
    double clawMult = 0.0005;
    double speedMult = 0.3;


    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        angle = hardwareMap.get(Servo.class, "angle");
        // angle.setPosition(anglePos);
        
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Instance of autonomous class
        AutoMethods auto = new AutoMethods(leftFront, leftRear, rightFront, rightRear, extend, angle, claw);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Get joystick inputs
            // X = gamepad1 = wheel movement
            double leftY = -gamepad1.left_stick_y; // Forward/Backward
            double leftX = -gamepad1.left_stick_x;  // Left/Right
            double rightX = gamepad1.right_stick_x; // Rotate

            double extendPower = gamepad2.left_stick_y; // extend
            double anglePower = gamepad2.right_stick_y * angleMult; // wrist
            
            if (gamepad2.left_bumper) // claw
                clawPower = 1;
            else if (gamepad2.right_bumper)
                clawPower = -1;
            else
                clawPower = 0;

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

            speedMult = 0.3 + (0.7*gamepad1.right_trigger);
            
            
            // Set wheel power
            leftFront.setPower(frontLeftPower * speedMult);
            leftRear.setPower(rearLeftPower * speedMult);
            rightFront.setPower(frontRightPower * speedMult);
            rightRear.setPower(rearRightPower * speedMult);

            // extend
            extend.setPower((extendPower * extendMult) + extend.getCurrentPosition()*0.000002);
            
            // Pick up specimen
            if (gamepad2.cross){
                anglePos = 0.5;
                clawPos = 0.28;
            }
            
            // Place specimen
            if (gamepad2.square){
                anglePos = 0.72;
                clawPos = 0.325;
            }
            
            if (gamepad1.triangle){
                auto.closeClaw();
                auto.wristAngle(0.6);
                sleep((long)1000.0);
                auto.extendUp(0.6 - (extend.getCurrentPosition() / 4250));
                auto.forward(18.0);
                auto.backward(6.0);
                auto.extendDown(0.1);
                sleep((long)100.0);
                auto.openClaw();
            }
            
            // wrist
            anglePos += anglePower;
            if (anglePos < 0.48)
                anglePos = 0.48;
            else if (anglePos > 0.85)
                anglePos = 0.85;
            angle.setPosition(anglePos);
            
            // claw
            clawPos += clawPower * clawMult;
            if (clawPos < 0.25)
                clawPos = 0.25;
            else if (clawPos > 0.325)
                clawPos = 0.325;
            claw.setPosition(clawPos);
            
            // Telemetry for debugging (optional)
            telemetry.addData("FL Power", frontLeftPower);
            telemetry.addData("RL Power", rearLeftPower);
            telemetry.addData("FR Power", frontRightPower);
            telemetry.addData("RR Power", rearRightPower);
            telemetry.addData("Extend Power", extendPower);
            telemetry.addData("Wrist Position", angle.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.update();
        }
    }

}
