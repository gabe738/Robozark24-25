package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class Kanye extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private void forward(double distance){
            /*
            Moves forward {distance} inches
             */
        double time = distance * 48; // approx 48 ms per inch
        leftFront.setPower(-0.3);
        leftRear.setPower(-0.3);
        rightFront.setPower(-0.3);
        rightRear.setPower(-0.3);

        sleep((long)time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }

    private void backward(double distance){
            /*
            Moves forward {distance} inches
             */
        double time = distance * 48; // approx 48 ms per inch
        leftFront.setPower(0.3);
        leftRear.setPower(0.3);
        rightFront.setPower(0.3);
        rightRear.setPower(0.3);

        sleep((long)time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }

    private void right(double angle){
        double time = angle*8.67; // approx 8.67 ms per degree

        leftFront.setPower(0.4);
        leftRear.setPower(0.4);
        rightFront.setPower(-0.4);
        rightRear.setPower(-0.4);

        sleep((long)time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }

    private void left(double angle){
        double time = angle*8.67; // approx 8.67 ms per degree

        leftFront.setPower(-0.4);
        leftRear.setPower(-0.4);
        rightFront.setPower(0.4);
        rightRear.setPower(0.4);

        sleep((long)time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }

    private void strafe(double time, int power){
        // if power positive strafe right

        leftFront.setPower(0.2*power);
        leftRear.setPower(-0.3*power);
        rightFront.setPower(-0.1*power);
        rightRear.setPower(0.2*power);

        sleep((long)time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }
    public void runOpMode(){
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Set motor directions if needed
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        // path of robot
        forward(42.0);

    }
}

