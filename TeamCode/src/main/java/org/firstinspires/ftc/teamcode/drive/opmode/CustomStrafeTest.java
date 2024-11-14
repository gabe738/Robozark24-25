package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous
public class CustomStrafeTest extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    public static double lfPower = 0.2;
    public static double lrPower = -0.3;
    public static double rfPower = -0.1;
    public static double rrPower = 0.2;
    public static int time = 100;
    public static double power = 0.3;
    // 0.3 48 ms per inch

    @Override
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

        // Right by default

        leftFront.setPower(0.2);
        leftRear.setPower(-0.3);
        rightFront.setPower(-0.1);
        rightRear.setPower(0.2);

        sleep(time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }
}

