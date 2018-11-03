package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ichigo on 9/15/2018.
 */

@TeleOp
public class TeleOp1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left;
    private DcMotor right;
    private Servo servo;
    private DcMotor collector;
    private DcMotor lifter;
    private DcMotor lifter2;


    //joystick-power mapping     https://www.desmos.com/calculator/r9ty28o3la
    final double b = 4;   //controls magnitude of the curve for joystick-power mapping
    final double a = Math.atan(b);   //constant for joystick-power mapping



    public void movement()
    {
        double forwardPower = this.gamepad1.right_stick_y * 0.75;
        double steeringPower = this.gamepad1.left_stick_x / 2;
        left.setPower(-(forwardPower-steeringPower));
        right.setPower((forwardPower+steeringPower));  //0.79
    }


    public void collect()
    {
        if(gamepad2.left_bumper) {
            collector.setPower(0.75);
        } else if(gamepad2.right_bumper) {
            collector.setPower(-0.75);
        } else {
            collector.setPower(0);
        }
    }

    public void lifting()
    {
        lifter.setPower(gamepad2.left_stick_y/2);
        //lifter2.setPower(gamepad2.right_stick_y);

        //lifter2.setPower(Math.pow(gamepad2.right_stick_y, 3));

        lifter2.setPower(-Math.tan(gamepad2.right_stick_y*a)/b);
    }

    public void resetlift()
    {

    }

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        collector = hardwareMap.get(DcMotor.class, "collector");
        lifter = hardwareMap.get(DcMotor.class, "lift1");
        lifter2 = hardwareMap.get(DcMotor.class, "lift2");


        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            collector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            movement();
            lifting();
            collect();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
