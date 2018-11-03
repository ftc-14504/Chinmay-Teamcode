package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This is NOT an opmode.
 *
 * This class is used to define all the specific navigation tasks for the Target Tracking Demo
 * It focuses on setting up and using the Vuforia Library, which is part of the 2016-2017 FTC SDK
 *
 * Once a target is identified, its information is displayed as telemetry data.
 * To approach the target, three motion priorities are created:
 * - Priority #1 Rotate so the robot is pointing at the target (for best target retention).
 * - Priority #2 Drive laterally based on distance from target center-line
 * - Priority #3 Drive forward based on the desired target standoff distance
 *
 */

public class RobotNav
{
    // Constants
    private static final int     MAX_TARGETS    =   4;
    private static final double  ON_AXIS        =  10;      // Within 1.0 cm of target center-line
    private static final double  CLOSE_ENOUGH   =  20;      // Within 2.0 cm of final target standoff

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.  Alt. is BACK
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;

    public  static final double  YAW_GAIN       =  0.018;   // Rate at which we respond to heading error
    public  static final double  LATERAL_GAIN   =  0.0027;  // Rate at which we respond to off-axis error
    public  static final double  AXIAL_GAIN     =  0.0017;  // Rate at which we respond to target distance errors

    /* Private class members. */
    private LinearOpMode        myOpMode;       // Access to the OpMode object
    private RobotDrv             myRobot;        // Access to the Robot hardware
    private VuforiaTrackables   targetsRoverRuckus;        // List of active targets

    // Navigation data is only valid if targetFound == true;
    private boolean             targetFound;    // set to true if Vuforia is currently tracking a target
    private String              targetName;     // Name of the currently tracked target
    private double              robotX;         // X displacement from target center
    private double              robotY;         // Y displacement from target center
    private double              robotBearing;   // Robot's rotation around the Z axis (CCW is positive)
    private double              targetRange;    // Range from robot's center to target in mm
    private double              targetBearing;  // Heading of the target , relative to the robot's unrotated center
    private double              relativeBearing;// Heading to the target from the robot's current bearing.
    //   eg: a Positive RelativeBearing means the robot must turn CCW to point at the target image.


    private static final String VUFORIA_KEY = "AQdDh+P/////AAAAGYG4khX9T0Mai5pYz9oTllp2KuZI24ZwM9ostcBXs2A90ddi/sJDOAabZEVM/5jhWNRN40BJ32nrSkbKTnqMnZ10v1A/PjDvnKwLG7zpA/wATnngFrhODfBwaHvP1WouKc+9f8QPOfLJnoGAlohWpfNWmdSe0UiyAeVoNCRW6TlLHECp85fs/acyk0eOy3qvUmJSFOTIsa5sJHVHscqpofheFgzhfmC7c+VUHGB8fIDiFBLdJBK9My1B2BBsJhblTZWgeVjOFI28qEHiEm7ADigF4zkH890YMfBRDr70ajPRJfOuzPAQA2QmOatQyL3tO/s9VmiIkcPDirMkTdwPbfBxUYkkCBGUDQMtYstBS58G";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    /* Constructor */
    public RobotNav(){

        targetFound = false;
        targetName = null;
        targetsRoverRuckus = null;

        robotX = 0;
        robotY = 0;
        targetRange = 0;
        targetBearing = 0;
        robotBearing = 0;
        relativeBearing = 0;
    }

    /***
     * Send telemetry data to indicate navigation status
     */
    public void addNavTelemetry() {
        if (targetFound)
        {
            // Display the current visible target name, robot info, target info, and required robot action.
            myOpMode.telemetry.addData("Visible", targetName);
            myOpMode.telemetry.addData("Robot", "[X]:[Y] (B) [%5.0fmm]:[%5.0fmm] (%4.0f°)",
                    robotX, robotY, robotBearing);
            myOpMode.telemetry.addData("Target", "[R] (B):(RB) [%5.0fmm] (%4.0f°):(%4.0f°)",
                    targetRange, targetBearing, relativeBearing);
            myOpMode.telemetry.addData("- Turn    ", "%s %4.0f°",  relativeBearing < 0 ? ">>> CW " : "<<< CCW", Math.abs(relativeBearing));
            myOpMode.telemetry.addData("- Strafe  ", "%s %5.0fmm", robotY < 0 ? "LEFT" : "RIGHT", Math.abs(robotY));
            myOpMode.telemetry.addData("- Distance", "%5.0fmm", Math.abs(robotX));
        }
        else
        {
            myOpMode.telemetry.addData("Visible", "- - - -" );
        }
    }


    /***
     * use target position to determine the best way to approach it.
     * Set the Axial, Lateral and Yaw axis motion values to get us there.
     *
     * @return true if we are close to target
     * @param standOffDistance how close do we get the center of the robot to target (in mm)
     */
    public boolean cruiseControl(double standOffDistance) {
        boolean closeEnough;

        // Priority #1 Rotate to always be pointing at the target (for best target retention).
        double Y  = (relativeBearing * YAW_GAIN);

        // Priority #2  Drive laterally based on distance from X axis (same as y value)
        double L  =(robotY * LATERAL_GAIN);

        // Priority #3 Drive forward based on the desiredHeading target standoff distance
        double A  = (-(robotX + standOffDistance) * AXIAL_GAIN);

        // Send the desired axis motions to the robot hardware.
        myRobot.setYaw(Y);
        myRobot.setAxial(A);
        myRobot.setLateral(L);

        // Determine if we are close enough to the target for action.
        closeEnough = ( (Math.abs(robotX + standOffDistance) < CLOSE_ENOUGH) &&
                (Math.abs(robotY) < ON_AXIS));

        return (closeEnough);
    }


    /***
     * Determine if specified target ID is visible and
     * If it is, retreive the relevant data, and then calculate the Robot and Target locations
     *
     * @param   targetId
     * @return  true if the specified target is found
     */
    public boolean targetIsVisible(int targetId) {

        VuforiaTrackable target = targetsRoverRuckus.get(targetId);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        OpenGLMatrix location  = null;

        // if we have a target, look for an updated robot position
        if ((target != null) && (listener != null) && listener.isVisible()) {
            targetFound = true;
            targetName = target.getName();

            // If we have an updated robot location, update all the relevant tracking information
            location  = listener.getUpdatedRobotLocation();
            if (location != null) {

                // Create a translation and rotation vector for the robot.
                VectorF trans = location.getTranslation();
                Orientation rot = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Robot position is defined by the standard Matrix translation (x and y)
                robotX = trans.get(0);
                robotY = trans.get(1);

                // Robot bearing (in +vc CCW cartesian system) is defined by the standard Matrix z rotation
                robotBearing = rot.thirdAngle;

                // target range is based on distance from robot position to origin.
                targetRange = Math.hypot(robotX, robotY);

                // target bearing is based on angle formed between the X axis to the target range line
                targetBearing = Math.toDegrees(-Math.asin(robotY / targetRange));

                // Target relative bearing is the target Heading relative to the direction the robot is pointing.
                relativeBearing = targetBearing - robotBearing;
            }
            targetFound = true;
        }
        else  {
            // Indicate that there is no target visible
            targetFound = false;
            targetName = "None";
        }

        return targetFound;
    }

    /***
     * Start tracking Vuforia images
     */
    public void activateTracking() {

        // Start tracking any of the defined targets
        if (targetsRoverRuckus != null)
            targetsRoverRuckus.activate();
    }

    /***
     * See if any of the vision targets are in sight.
     *
     * @return true if any target is found
     */
    public boolean targetsAreVisible()  {

        int targetTestID = 0;

        // Check each target in turn, but stop looking when the first target is found.
        while ((targetTestID < MAX_TARGETS) && !targetIsVisible(targetTestID)) {
            targetTestID++ ;
        }

        return (targetFound);
    }

    /***
     * Initialize the Target Tracking and navigation interface
     * @param opMode    pointer to OpMode
     * @param robot     pointer to Robot hardware class
     */

    public void initVuforia(LinearOpMode opMode, RobotDrv robot) {

        // Save reference to OpMode and Hardware map
        myOpMode = opMode;
        myRobot = robot;

        /**
         * Start up Vuforia, telling it the id of the view that we wish to use as the parent for
         * the camera monitor.
         * We also indicate which camera on the RC that we wish to use.
         */

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);  // Use this line to see camera display
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();                             // OR... Use this line to improve performance

        // Get your own Vuforia key at  https://developer.vuforia.com/license-manager
        // and paste it here...
        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = false;
        VuforiaLocalizer vuforia;// = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data sets that for the trackable objects we wish to track.
         * These particular data sets are stored in the 'assets' part of our application
         * They represent the four image targets used in the 2016-17 FTC game.
         */

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
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
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
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
         * pointing to the LEFT side of the Robot.  If we consider that the camera and screen will be
         * in "Landscape Mode" the upper portion of the screen is closest to the front of the robot.
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

        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // Camera is ON the robots center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, CAMERA_CHOICE == VuforiaLocalizer.CameraDirection.FRONT ? 90 : -90, 0, 0));


        // Set the all the targets to have the same location and camera orientation
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }
    }
}
