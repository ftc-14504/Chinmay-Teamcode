package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class Collector extends LinearOpMode{
    private DcMotor collector;
    private DcMotor left;
    public void runOpMode(){
        collector = hardwareMap.get(DcMotor.class, "collector");
        left = hardwareMap.get(DcMotor.class, "left");
        double tgtPower = 0;
        while (opModeIsActive()) {
            tgtPower = -this.gamepad1.left_stick_y;
            collector.setPower(tgtPower);
            left.setPower(this.gamepad1.right_stick_y);
        }

    }
}
