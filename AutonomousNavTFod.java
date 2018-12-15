package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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

/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="AutonomousNavTFod", group ="Autonomous")

public class AutonomousNavTFod extends LinearOpMode {

    private DcMotor right;
    private DcMotor right2;
    private DcMotor left;
    private DcMotor left2;
    private Servo marker;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 2240;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 3.0/4.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 100/25.4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.3;
    boolean amIDone = false;

    public double robotX;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    private static final String VUFORIA_KEY = "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeVoNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzhfmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7ADigF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDirMkTdwPbfBxUYkkCBGUDQMtYstBS58G";


    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmFTCFieldWidth = (12 * 6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private boolean targetVisible = false;
    VuforiaLocalizer vuforia;

    RobotNav rnav;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive ( double speed,
                               double leftInches, double rightInches,
                               double timeoutS){
        int newLeftTarget, newLeft2Target;
        int newRightTarget, newRight2Target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int) ((leftInches - 0.3) * COUNTS_PER_INCH);
            newLeft2Target = left2.getCurrentPosition() + (int) ((leftInches - 0.3) * COUNTS_PER_INCH);

            newRightTarget = right.getCurrentPosition() + (int) ((rightInches - 0.3) * COUNTS_PER_INCH);
            newRight2Target = right2.getCurrentPosition() + (int) ((rightInches - 0.3) * COUNTS_PER_INCH);

            left.setTargetPosition(newLeftTarget);
            left2.setTargetPosition(newLeft2Target);
            right.setTargetPosition(newRightTarget);
            right2.setTargetPosition(newRight2Target);

            // Turn On RUN_TO_POSITION
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
                    (left.isBusy()  && left2.isBusy() && right.isBusy() && right2.isBusy())) {

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
        encoderDrive(TURN_SPEED, (Math.PI/30.0)*angle, (-Math.PI/30.0)*angle, 5.0);
    }

    public void depositMarker() {
        while(marker.getPosition() < 0.6) {
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

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        marker.setPosition(marker.getPosition()+0.001);

        waitForStart();

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();

        int tfodCounter = 0;

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }

        while (opModeIsActive() && !amIDone) {
            int picture_found = 7;
            int turnPos = 0;
            float turnIncrement = 24;
            float DriveInc = 30;

            // first detect the mineral
            List<Recognition> minerals = mineralDetection(3.0);
            int mineralPosition = threeInARow(minerals);

            List<Recognition> filteredMinerals = minerals;    //new list of minerals detected, without ones in the background
            for (int i = 0; i < filteredMinerals.size(); i++) {
                if((filteredMinerals.get(i).getRight() - filteredMinerals.get(i).getLeft() < 100) || (filteredMinerals.get(i).getBottom() < 800)) { //if the size of the box is less than 100, then it's in the background so its removed
                    filteredMinerals.remove(i);
                }
            }


            // otherwise see what was detected and do different cases

            if (filteredMinerals != null) {
                switch (filteredMinerals.size()) {
                    case 1: //only one mineral detected
                        telemetry.addData("Position: ","left = %7f, right = %7f, difference = %7f", filteredMinerals.get(0).getLeft(), filteredMinerals.get(0).getRight(), filteredMinerals.get(0).getRight()-filteredMinerals.get(0).getLeft());
                        telemetry.addData("Bottom", filteredMinerals.get(0).getBottom());
                        telemetry.update();

                        if (filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) || turnPos >= 2) {
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            telemetry.addLine("Found it!");
                            telemetry.update();
                            amIDone = true;
                            break;
                        }
                        else {
                            //turn right a bit
                            turnDegrees(turnIncrement);
                            telemetry.addLine("turn right");
                            telemetry.update();
                            sleep(1000);
                            turnPos++;
                            telemetry.addData("turnPos: ", turnPos);
                            break;
                        }

                    case 2: //two minerals detected
                        if((filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(0).getLeft() < 200) || (filteredMinerals.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(1).getLeft() < 200)) { //the left mineral is gold
                            //turn towards the gold mineral seen (on the left) and drive forward
                            turnDegrees(-0.5*turnIncrement);
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            telemetry.addLine("mineral at relative left");
                            telemetry.update();
                            amIDone = true;
                            break;
                        }
                        if((filteredMinerals.get(0).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(0).getLeft() >= 200) || (filteredMinerals.get(1).getLabel().equals(LABEL_GOLD_MINERAL) && filteredMinerals.get(1).getLeft() >= 200)) { //the right mineral is gold
                            //turn towards the gold mineral seen (on the right) and drive forward
                            turnDegrees(turnIncrement * 0.5);
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            telemetry.addLine("mineral at relative right");
                            telemetry.update();
                            amIDone = true;
                            break;
                        }
                        if (filteredMinerals.get(0).getLabel().equals(LABEL_SILVER_MINERAL) && filteredMinerals.get(1).getLabel().equals(LABEL_SILVER_MINERAL)) { //none of the two visible minerals are gold. So, the third one must be gold
                            //turn way right and drive forward
                            turnDegrees(turnIncrement*1.5);
                            moveForward(DriveInc);
                            moveForward(-DriveInc);
                            telemetry.addLine("mineral is all the way to the right");
                            telemetry.update();
                            amIDone = true;
                            break;
                        }
                }
            }
            continue;

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

            if (counter==0 ) {
                turnDegrees(-50);
                moveForward(35.0);
                counter++;
            }

            switch (picture_found) {
                case 0: //Blue rover. So we are in blue alliance, crater is to the right, and depot to the left
                    // so go forward 3ft
                    // turn 90 degrees left,
                    // cruise forward 5ft,
                    // deposit marker
                    // go back 12ft

                    //moveForward(35-inchForward);
                    turnDegrees(-86);
                    moveForward(54);
                    depositMarker();
                    turnDegrees(-7);
                    moveForward(-122);
                    amIDone = true;

                    break;
                case 1: //red footprint: So we are in red alliance, crater to the right, and depot is to the left
                    //moveForward(35-inchForward);
                    turnDegrees(-86);
                    moveForward(54);
                    depositMarker();
                    turnDegrees(-7);
                    moveForward(-122);
                    amIDone = true;
                    break;
                case 2: // front craters: So we are in blue alliance, depot to the right, crater to the right back
                    //moveForward(35-inchForward);
                    turnDegrees(88);
                    moveForward(40);
                    turnDegrees(-91.5);
                    depositMarker();
                    moveForward(-122);
                    amIDone = true;
                    break;
                case 3:// back space: So we are in red alliance, depot to the right, crater to the right back
                    //moveForward(35-inchForward);
                    turnDegrees(88);
                    moveForward(40);
                    turnDegrees(-91.5);
                    depositMarker();
                    moveForward(-122);
                    amIDone = true;
                    break;
                default:
                    /*
                    if (counter % 2 == 1 ) {
                        inchForward += 4.0;
                        moveForward(4.0);
                    }*/
            /*
                    turnDegrees(5);
                    turnDegrees(-5);
                    break;
                //}
            }

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
        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }

    public VuforiaTrackables setUpVuforia(List<VuforiaTrackable> allTrackables) {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        //List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

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

}