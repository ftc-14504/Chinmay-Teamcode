package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by ichigo on 9/15/2018.
 */

@TeleOp
public class TeleOp2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fLeft;
    private DcMotor bLeft;
    private DcMotor fRight;
    private DcMotor bRight;
    private DcMotor lift;

    //joystick-power mapping     https://www.desmos.com/calculator/r9ty28o3la
    final double b = 2;   //controls magnitude of the curve for joystick-power mapping
    final double a = Math.atan(b);   //constant for joystick-power mapping




    public double clipPower(double x) {  //a is value, b is lower end of range, c is higher end of range
        if(x < -1) x = -1;
        if(x > 1) x = 1;
        return x;
    }



    //----------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------

    int dirToggle = 1;
    int turnMode = 0;

    double angle = 0.0;

    double fl2 = 1.0;
    double fr2 = 1.0;
    double bl2 = 1.0;
    double br2 = 1.0;

    double ATAN1 = Math.atan(1.0);
    double SQRT2 = Math.sqrt(2.0);

    boolean firstTime = false;

    public void movement()
    {
        telemetry.addData("x = ", gamepad1.right_stick_x);
        telemetry.addData("y = ", gamepad1.right_stick_y);
        telemetry.addData("trigger = ", gamepad1.right_trigger);
        telemetry.addData("angle = ", angle);


        if(gamepad1.left_bumper) {
            dirToggle = 1;
        }
        if(gamepad1.right_bumper) {
            dirToggle = -1;
        }

        if(gamepad1.a) {
            turnMode = 1;  //turning around a point that's not the center
        } else if(gamepad1.x){
            turnMode = 2;  //turning while still moving in a straight line
        } else {
            turnMode = 0;  // normal movement and turning around center
            angle = 0.0;
        }

        if(turnMode == 0) {
            double axialPower = -this.gamepad1.right_stick_y;
            double lateralPower = this.gamepad1.right_stick_x;
            double turnPower = this.gamepad1.left_stick_x * -dirToggle;

            double fl = (axialPower + lateralPower) / 2;
            double fr = (axialPower - lateralPower) / 2;
            double bl = (axialPower - lateralPower) / 2;
            double br = (axialPower + lateralPower) / 2;


            fl += turnPower;
            fr -= turnPower;
            bl += turnPower;
            br -= turnPower;

            fLeft.setPower(dirToggle * fl);
            fRight.setPower(dirToggle * fr);
            bLeft.setPower(dirToggle * bl);
            bRight.setPower(dirToggle * br);
        } else if(turnMode == 1){
            //left joystick controls point to turn around
            //triggers control actually turning
        } else if(turnMode == 2) {
            //left joystick controls speed of movement (direction remains the same)
            //triggers control the turning

            //double forward = gamepad1.left_stick_y;
            double forward = 1;
            double turnPower;
            if(gamepad1.right_trigger > 0) {
                turnPower = gamepad1.right_trigger;
            } else if (gamepad1.left_trigger > 0) {
                turnPower = -gamepad1.left_trigger;
            } else {
                turnPower = 0.0;
            }
            angle += turnPower / 5.0;

            //https://www.desmos.com/calculator/nubffvwvt2
            fl2 = clipPower((Math.toRadians(-angle) - ATAN1) * SQRT2);
            fr2 = clipPower((Math.toRadians(-angle) + ATAN1) * SQRT2);
            br2 = clipPower((Math.toRadians(-angle) - ATAN1) * SQRT2);
            bl2 = clipPower((Math.toRadians(-angle) + ATAN1) * SQRT2);



            fLeft.setPower(fl2);
            fRight.setPower(fr2);
            bRight.setPower(br2);
            bLeft.setPower(bl2);

            telemetry.addData("turnPower = ", turnPower);

        }
    }

    //----------------------------------------------------------------------------------------------------
    //----------------------------------------------------------------------------------------------------


    public void collect()
    {

    }

    public void lifting()
    {
        lift.setPower(gamepad2.right_stick_y);
    }



    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fLeft = hardwareMap.get(DcMotor.class, "fLeft");
        bLeft = hardwareMap.get(DcMotor.class, "bLeft");
        fRight = hardwareMap.get(DcMotor.class, "fRight");
        bRight = hardwareMap.get(DcMotor.class, "bRight");
        lift = hardwareMap.get(DcMotor.class, "liftest");

        fLeft.setDirection(DcMotor.Direction.REVERSE);
        bLeft.setDirection(DcMotor.Direction.REVERSE);
        fRight.setDirection(DcMotor.Direction.FORWARD);
        bRight.setDirection(DcMotor.Direction.FORWARD);





        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            movement();
            lifting();
            collect();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
