package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous
public class CustomTurnTest extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    public static int time = 100;
    public static double power = 0.5;
    // at 0.4 power, 780 ms is 90 degrees

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

        leftFront.setPower(power);
        leftRear.setPower(power);
        rightFront.setPower(power*-1);
        rightRear.setPower(power*-1);

        sleep(time);

        leftFront.setPower(0.0);
        leftRear.setPower(0.0);
        rightFront.setPower(0.0);
        rightRear.setPower(0.0);
    }
}

