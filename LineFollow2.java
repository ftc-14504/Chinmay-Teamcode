package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.media.SoundPool;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by chinmay on 9/1/17.
 */
@Autonomous
public class LineFollow2 extends LinearOpMode {
    ColorSensor color_sensor;
    final double SPEED = 0.1;
    final double EDGE_COLOR = 4000;
    double left, right, correct;
    private DcMotor LeftPow, RightPow;

    // Sound variables
    public SoundPool mySound;
    public int sndId;


    @Override
    public void runOpMode() {

        color_sensor = hardwareMap.colorSensor.get("color");
        color_sensor.enableLed(true);
        LeftPow = hardwareMap.dcMotor.get("left");
        RightPow = hardwareMap.dcMotor.get("right");

        //MediaPlayer mediaPlayer = MediaPlayer.create(this, R.raw.serenity_now);
        //mediaPlayer.start();

        //setup the sounds
        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        sndId = mySound.load(hardwareMap.appContext, R.raw.serenity, 1); // PSM


        waitForStart();

        double max_color = 0;

        mySound.play(sndId,1,1,1,0,1);

        while (true) {

            if (color_sensor.alpha() > max_color) max_color = color_sensor.alpha();

            correct = (EDGE_COLOR - color_sensor.alpha())/(max_color*(0.5/SPEED));

            if (correct <= 0) {
                left = SPEED - correct;
                right = SPEED;
            } else {
                left = SPEED;
                right = SPEED + correct;
            }

            LeftPow.setPower(-left);
            RightPow.setPower(right);

            telemetry.addData("Light Level: ", color_sensor.alpha());
            telemetry.addData("Max Light Level: ", max_color);
            telemetry.update();
        }
    }
}


