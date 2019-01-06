package org.firstinspires.ftc.teamcode.FTCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;
@Autonomous(name="AutonomousNavTFod", group ="Autonomous")

public class AutonomousNavTFod extends LinearOpMode {

    private DcMotor right;
    private DcMotor right2;
    private DcMotor left;
    private DcMotor left2;
    private Servo marker;
    private DcMotor arm;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 3.0 / 4.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 100 / 25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;
    boolean amIDone = false;
    boolean PictureClose = false;


    public double robotX;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;
    private static final String VUFORIA_KEY = "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeVoNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzhfmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7ADigF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDirMkTdwPbfBxUYkkCBGUDQMtYstBS58G";
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor
    VuforiaLocalizer vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();


    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;

    RobotNav rnav;

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget, newLeft2Target;
        int newRightTarget, newRight2Target;


        if (opModeIsActive()) {

            newLeftTarget = left.getCurrentPosition() + (int) ((leftInches - 0.3) * COUNTS_PER_INCH);
            newLeft2Target = left2.getCurrentPosition() + (int) ((leftInches - 0.3) * COUNTS_PER_INCH);

            newRightTarget = right.getCurrentPosition() + (int) ((rightInches - 0.3) * COUNTS_PER_INCH);
            newRight2Target = right2.getCurrentPosition() + (int) ((rightInches - 0.3) * COUNTS_PER_INCH);

            left.setTargetPosition(newLeftTarget);
            left2.setTargetPosition(newLeft2Target);
            right.setTargetPosition(newRightTarget);
            right2.setTargetPosition(newRight2Target);

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left.setPower(Math.abs(speed));
            left2.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));
            right2.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left.isBusy() && left2.isBusy() && right.isBusy() && right2.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        left.getCurrentPosition(),
                        left2.getCurrentPosition(),
                        right.getCurrentPosition(),
                        right2.getCurrentPosition());

                /*
                if (rnav.targetIsVisible(0)){
                    double distance = rnav.robotX;
                    telemetry.addData("Distance from target, %7d", distance);
                }*/

                telemetry.update();


            }
            left.setPower(0);
            left2.setPower(0);

            right.setPower(0);
            right2.setPower(0);

            // Turn OFF RUN_TO_POSITION

            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void moveForward(double distance) {
        encoderDrive(DRIVE_SPEED, -distance, -distance, 5.0);
    }

    public void turnDegrees(double angle) {
        encoderDrive(TURN_SPEED, (Math.PI / 30.0) * angle, (-Math.PI / 30.0) * angle, 5.0);
    }

    public void depositMarker() {
        while (marker.getPosition() < 0.6) {
            marker.setPosition(marker.getPosition() + 0.001);
        }
        sleep(300);
        marker.setPosition(0.42);
    }

    public void initDrive() {
        left = hardwareMap.get(DcMotor.class, "fLeft");
        left2 = hardwareMap.get(DcMotor.class, "bLeft");
        right = hardwareMap.get(DcMotor.class, "fRight");
        right2 = hardwareMap.get(DcMotor.class, "bRight");
        marker = hardwareMap.get(Servo.class, "marker");
        arm = hardwareMap.get(DcMotor.class, "arm");
        left.setDirection(DcMotor.Direction.FORWARD);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
    }

    private int threeInARow(List<Recognition> updatedRecognitions) {

        int rval = -1;
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            if (updatedRecognitions.size() == 3) {
                int goldMineralX = -1;
                int silverMineral1X = -1;
                int silverMineral2X = -1;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    } else if (silverMineral1X == -1) {
                        silverMineral1X = (int) recognition.getLeft();
                    } else {
                        silverMineral2X = (int) recognition.getLeft();
                    }
                }
                if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                    if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Left");
                        rval = 0; // gold is leftmost
                    } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                        telemetry.addData("Gold Mineral Position", "Right");
                        rval = 2;//rightmost
                    } else {
                        telemetry.addData("Gold Mineral Position", "Center");
                        rval = 1; //middle
                    }
                }
            }
            telemetry.update();
        }

        return rval;
    }

    public List<Recognition> mineralDetection(double timeout) {
        int rval = -1;
        List<Recognition> updatedRecognitions = null;
        if (tfod != null) {

            runtime.reset();

            while (updatedRecognitions == null && runtime.seconds() < timeout) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                updatedRecognitions = tfod.getUpdatedRecognitions();
            }


        }

        return updatedRecognitions;

    }

    public VuforiaTrackables setUpVuforia(List<VuforiaTrackable> allTrackables) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");


        allTrackables.addAll(targetsRoverRuckus);


        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);


        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);


        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT = 180;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 220;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT = 130;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
        return targetsRoverRuckus;
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    @Override
    public void runOpMode() {

        initDrive(); //initialize motors
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

        VuforiaTrackables targetsRoverRuckus = setUpVuforia(allTrackables); // setup vuforia

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        int turnPos = 0;
        int tfodCounter = 0;
        boolean triggered = false;
        boolean trigger = false;
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        marker.setPosition(marker.getPosition() + 0.001);

        waitForStart();

        targetsRoverRuckus.activate();


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }


        while (opModeIsActive() && !amIDone) {


            float turnIncrement = 37;
            float DriveInc = 30;

            // first detect the mineral
            List<Recognition> minerals = mineralDetection(3.0);
            int mineralPosition = threeInARow(minerals);

            List<Recognition> filteredMinerals = minerals;    //new list of minerals detected, without ones in the background
            for (int i = 0; i < filteredMinerals.size(); i++) {
                if ((filteredMinerals.get(i).getRight() - filteredMinerals.get(i).getLeft() < 100) || (filteredMinerals.get(i).getBottom() < 700)) { //if the size of the box is less than 100, then it's in the background so its removed
                    filteredMinerals.remove(i);                                                                 //orinally 800
                }
            }


            // otherwise see what was detected and do different cases

            if (filteredMinerals != null) {
                switch (filteredMinerals.size()) {
                    case 1: //only one mineral detected
                        telemetry.addData("Position: ", "left = %7f, right = %7f, difference = %7f", filteredMinerals.get(0).getLeft(), filteredMinerals.get(0).getRight(), filteredMinerals.get(0).getRight() - filteredMinerals.get(0).getLeft());
                        telemetry.addData("Bottom", filteredMinerals.get(0).getBottom());
                        telemetry.update();

                        if (filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) || turnPos >= 2) {
                            moveForward(DriveInc);
                            moveForward(-DriveInc);

                            triggered = true;
                            telemetry.addLine("Found it!");
                            telemetry.update();
                            amIDone = true;
                            break;
                        } else {
                            //turn right a bit
                            turnDegrees(turnIncrement);
                            telemetry.addLine("turn right");
                            telemetry.update();
                            sleep(800);
                            turnPos++;
                            telemetry.addData("turnPos: ", turnPos);
                            break;
                        }

                    case 2: //two minerals detected
                        if ((filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(0).getLeft() < 200) || (filteredMinerals.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(1).getLeft() < 200)) { //the left mineral is gold
                            //turn towards the gold mineral seen (on the left) and drive forward
                            turnDegrees(-0.5 * turnIncrement);
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            triggered = true;
                            telemetry.addLine("mineral at relative left");
                            telemetry.update();
                            amIDone = true;
                            turnPos -= 0.5;
                            break;
                        }
                        if ((filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(0).getLeft() >= 200) || (filteredMinerals.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(1).getLeft() >= 200)) { //the right mineral is gold
                            //turn towards the gold mineral seen (on the right) and drive forward
                            turnDegrees(turnIncrement * 0.5);

                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            triggered = true;
                            telemetry.addLine("mineral at relative right");
                            telemetry.update();
                            amIDone = true;
                            turnPos += 0.5;
                            break;
                        }
                        if (filteredMinerals.get(0).getLabel().equals(LABEL_SILVER_MINERAL) && filteredMinerals.get(1).getLabel().equals(LABEL_SILVER_MINERAL)) { //none of the two visible minerals are gold. So, the third one must be gold
                            //turn way right and drive forward
                            turnDegrees(turnIncrement * 1.5);
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            triggered = true;
                            telemetry.addLine("mineral is all the way to the right");
                            telemetry.update();
                            amIDone = true;
                            turnPos += 1.5;
                            break;
                        }
                }
            }
            if (triggered) {
                telemetry.addLine("its alive");
                turnDegrees(-140 + (turnIncrement * (2 - turnPos)));
                moveForward(40);
                turnDegrees(-1);
            }
            int picture_found = 7;
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;

                    for (int i = 0; i < 4; i++) {
                        if (targetsRoverRuckus.get(i) == trackable) {
                            // 0 = blue rover, 1 = red footprint, 2 = front craters, 3 = back space
                            picture_found = i;
                        }
                    }

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();

                /*while (translation.get(2) < 6){
                    left.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                PictureClose = true;*/


            //if (PictureClose) {
                /*VectorF translation = lastLocation.getTranslation();
                while (translation.get(1) != 7) {
                    moveForward(translation.get(1) - 7);
                }
                while ((-2 >= translation.get(0)) || (translation.get(0) >= 2)) {
                    if (-2 >= translation.get(0)) {
                        turnDegrees(3);
                    } else if (translation.get(0) >= 2) {
                        turnDegrees(-3);
                    }
                }
                trigger = true;
            }
            telemetry.addData("TriGGer", trigger);
            telemetry.update();
            if (!trigger) {
                telemetry.addData("OWO", 69);
                telemetry.update();
            }
            if (trigger) {*/
                /*if(picture_found==1) {
                    turnDegrees(-79);
                    moveForward(54);
                    depositMarker();
                    turnDegrees(-7);
                    moveForward(-122);
                    amIDone = true;
                    break;
                }
                if(picture_found==2) {
                    turnDegrees(-79);
                    moveForward(54);
                    depositMarker();
                    turnDegrees(-7);
                    moveForward(-122);
                    amIDone = true;
                    break;
                }*/

            if (targetVisible) {
                switch (picture_found) {
                    case 0:

                        turnDegrees(-89);
                        moveForward(54);
                        depositMarker();
                        turnDegrees(-7);
                        moveForward(-122);
                        amIDone = true;
                        break;
                    case 1: //red footprint: So we are in red alliance, crater to the right, and depot is to the left
                        //Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        //turnDegrees(-(rotation.thirdAngle));
                        //
                        telemetry.addData("redfootprintfound", picture_found);
                        telemetry.update();
                        turnDegrees(-135);//try -138
                        moveForward(54);
                        depositMarker();
                        turnDegrees(-2);
                        moveForward(-122);
                        amIDone = true;
                        break;
                    case 2: // front craters: So we are in blue alliance, depot to the right, crater to the right back]
                        Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                        turnDegrees(-(rotation.thirdAngle));
                        //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                        //turnDegrees(98); //could be 85
                        moveForward(42);
                        turnDegrees(-160);//could comment this out for red crater

                        depositMarker();
                        moveForward(-122);
                        amIDone = true;
                        break;
                }
            }
        }
    }
}
                            /*Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                            turnDegrees(-(rotation.thirdAngle));

                        case 3:// back space: So we are in red alliance, depot to the right, crater to the right back
                            turnDegrees(98); //could be 85
                            moveForward(40);
                            turnDegrees(-89);//could comment this out for red crater
                            depositMarker();
                            moveForward(-122);
                            amIDone = true;
                            break;
                        default:
                            turnDegrees(5);
                            turnDegrees(-5);
                            break;

                    }

                    if (tfod != null) {
                        tfod.shutdown();
                    }
                }

            }
        }
    }






















 /*
            switch (mineralPosition) {
                case 0:
                    // left position mineral
                    turnDegrees(-20);
                    moveForward(12);
                    moveForward(-12);
                    break;
                case 1:
                    moveForward(10);
                    moveForward(-10);
                    break;
                case 2:
                    turnDegrees(20);
                    moveForward(12);
                    moveForward(-12);
                    break;
                case -1:
                    tfodCounter++;
                    //telemetry.addData("tfodcounter = ", tfodCounter);
                    //telemetry.update();
                    continue;
            }
            // check all the trackable target to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    for (int i = 0; i < 4; i++) {
                        if (targetsRoverRuckus.get(i) == trackable) {
                            // 0 = blue rover, 1 = red footprint, 2 = front craters, 3 = back space
                            picture_found = i;
                        }
                    }
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                        // Create a translation and rotation vector for the robot.
                    }
                    break;
                }
            }
            //rnav = new RobotNav();
            //rnav.initVuforia(this);
            //rnav.activateTracking();
            //double distance = rnav.cruiseControl(152.4);
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
            */
