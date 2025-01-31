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
public class Kanye extends LinearOpMode {
    double overallDistanceModifier = 38.0;
    double theTurnAdjustor = 6.5;

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
        
        // specimen
        sleep((long)200.0);
        auto.wristAngle(0.6);
        auto.closeClaw();
        sleep((long)400.0);
        auto.forward(8.0);
        sleep((long)300.0);
        auto.left(90.0);
        auto.forward(18.0);
        sleep((long)300.0);
        auto.right(90.0);
        auto.backward(15.0);
        auto.extendUp(0.5);
        auto.forward(32.0);
        auto.extendDown(0.1);
        sleep((long)100.0);
        auto.openClaw();
        auto.forward(8.0);
        auto.backward(52.0);
        
        // park
        auto.forward(4.0);
        sleep((long)300.0);
        auto.right(100.0);
        auto.forward(55.0);
        
    }
}

