package org.firstinspires.ftc.teamcode;

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
public class AutoMethods extends LinearOpMode {
    public double overallDistanceModifier = 38.0;
    public double theTurnAdjustor = 6.5;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;
    public DcMotorEx extend;
    public Servo wrist;
    public Servo claw;
    public double speed = 0.3;
    
    public void extendUp(double distance){
        
        // extends by a percentage, 1.0 is all the way up
        
        Boolean running = true;
        while (running) {
            running = false;
            if (extend.getCurrentPosition() < distance*4250) {
                extend.setPower(speed*1.3);
                running = true;
            }

        }
        extend.setPower(0.0);
    }
    
    public void extendDown(double distance){
        
        // extends by a percentage, 1.0 is all the way up
        
        Boolean running = true;
        while (running) {
            running = false;
            if (extend.getCurrentPosition() > distance*4250) {
                extend.setPower(-speed*1.3);
                running = true;
            }

        }
        extend.setPower(0.0);
    }
    
    public void wristAngle(double distance){
        
        // extends by a percentage, 1.0 is all the way up
        
        wrist.setPosition(0.45 + (0.45*distance));
    }
    
    public void openClaw(){
        claw.setPosition(0.275);
    }
    
    public void closeClaw(){
        claw.setPosition(0.325);
    }

    public void forward(double distance){
        
        //  Moves forward {distance} inches
        
        double distanceRatio = overallDistanceModifier;
        int[] initialMotorPositions = {leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftRear.getCurrentPosition(),rightRear.getCurrentPosition()};
        double lowestPosition = 0.0;
        Boolean running = true;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() < (initialMotorPositions[0] + (distance * distanceRatio))) {
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                running = true;
            }
            
            if (leftRear.getCurrentPosition() < (initialMotorPositions[2] + (distance * distanceRatio))) {
                leftRear.setPower(speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() < (initialMotorPositions[3] + (distance * distanceRatio))) {
                rightRear.setPower(speed);
                running = true;
            }

        }
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public void backward(double distance){
        
        // moves backward {distance} inches

        double distanceRatio = overallDistanceModifier;
        distance = -distance;
        int[] initialMotorPositions = {leftFront.getCurrentPosition(),rightFront.getCurrentPosition(),leftRear.getCurrentPosition(),rightRear.getCurrentPosition()};
        Boolean running = true;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() > (initialMotorPositions[0] + (distance * distanceRatio))) {
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                running = true;
            }
            if (leftRear.getCurrentPosition() > (initialMotorPositions[2] + (distance * distanceRatio))) {
                leftRear.setPower(-speed);
                running = true;
            }
            if (rightRear.getCurrentPosition() > (initialMotorPositions[3] + (distance * distanceRatio))) {
                rightRear.setPower(-speed);
                running = true;
            }

        }
        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public void right(double angle){
        
        // turns right {angle} degrees

        double turnRatio = theTurnAdjustor;
        int[] initialMotorPosition = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        boolean running = true;
        double offset = angle * turnRatio;
        double turnSpeed = 0.75;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() < (offset + initialMotorPosition[0])) {
                leftFront.setPower(turnSpeed);
                running = true;
            }
            if (leftRear.getCurrentPosition() < (offset + initialMotorPosition[2])) {
                leftRear.setPower(turnSpeed);
                running = true;
            }
            if (rightRear.getCurrentPosition() > (-offset + initialMotorPosition[3])) {
                rightRear.setPower(-2*turnSpeed);
                rightFront.setPower(-2*turnSpeed);
                running = true;
            }
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public void left(double angle){
        
        // turns left {angle} degrees

        double turnRatio = theTurnAdjustor;
        int[] initialMotorPosition = {leftFront.getCurrentPosition(), rightFront.getCurrentPosition(), leftRear.getCurrentPosition(), rightRear.getCurrentPosition()};
        boolean running = true;
        double offset = angle * turnRatio;
        double turnSpeed = 0.75;
        while (running) {
            running = false;
            if (leftFront.getCurrentPosition() > (-offset + initialMotorPosition[0])) {
                leftFront.setPower(-turnSpeed);
                running = true;
            }
            if (leftRear.getCurrentPosition() > (-offset + initialMotorPosition[2])) {
                leftRear.setPower(-turnSpeed);
                running = true;
            }
            if (rightRear.getCurrentPosition() < (offset + initialMotorPosition[3])) {
                rightRear.setPower(2*turnSpeed);
                rightFront.setPower(2*turnSpeed);
                running = true;
            }
        }

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightRear.setPower(0.0);
    }

    public AutoMethods(DcMotorEx lf, DcMotorEx lr, DcMotorEx rf, DcMotorEx rr, DcMotorEx e, Servo w, Servo c) {
        leftFront = lf;
        leftRear = lr;
        rightFront = rf;
        rightRear = rr;
        extend = e;
        wrist = w;
        claw = c;
    }

    public void runOpMode(){
        
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        
        wrist = hardwareMap.get(Servo.class, "angle");
        // wrist.setPosition(0.45);
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
        
        
    }
}

