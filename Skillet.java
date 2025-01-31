package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.AutoMethods;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class Skillet extends LinearOpMode {
    double overallDistanceModifier = 38.0;
    double theTurnAdjustor = 6.3;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx extend;
    private Servo wrist;
    private Servo claw;
    private double speed = 0.3;
    
    public void runOpMode(){
        
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        
        // Set encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Turn the motor back on when we are done
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        wrist = hardwareMap.get(Servo.class, "angle");
        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        
        // motors brake when power is cut
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        AutoMethods auto = new AutoMethods(leftFront, leftRear, rightFront, rightRear, extend, wrist, claw);

        waitForStart();

        /*
            Path of the Robot. Available functions:
            forward(inches)
            backward(inches)
            right(degrees)
            left(degrees)
            extend(percent) ex. 0.75
            wristAngle(percent)
            openClaw()
            closeClaw()
        */
        
        // Park
        
        sleep((long)100.0);
        auto.wristAngle(0.6);
        sleep((long)200.0);
        auto.forward(5.0);
        sleep((long)300.0);
        auto.right(95.0);
        auto.forward(12.0);
        sleep((long)1000.0);
        auto.forward(12.0);
        sleep((long)1000.0);
        auto.forward(5.0);
        
    }
}

