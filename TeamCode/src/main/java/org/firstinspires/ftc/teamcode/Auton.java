package org.firstinspires.ftc.teamcode;

// Imports
// RoadRunner Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;

// FTCLib imports
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

// FTC SDK imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TimedAction;

// OpenCV imports
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Template Autonomous")
public class Auton extends LinearOpMode {
    /*
    $$$$$$$\                      $$\                               $$\     $$\
    $$  __$$\                     $$ |                              $$ |    \__|
    $$ |  $$ | $$$$$$\   $$$$$$$\ $$ | $$$$$$\   $$$$$$\  $$$$$$\ $$$$$$\   $$\  $$$$$$\  $$$$$$$\   $$$$$$$\
    $$ |  $$ |$$  __$$\ $$  _____|$$ | \____$$\ $$  __$$\ \____$$\\_$$  _|  $$ |$$  __$$\ $$  __$$\ $$  _____|
    $$ |  $$ |$$$$$$$$ |$$ /      $$ | $$$$$$$ |$$ |  \__|$$$$$$$ | $$ |    $$ |$$ /  $$ |$$ |  $$ |\$$$$$$\
    $$ |  $$ |$$   ____|$$ |      $$ |$$  __$$ |$$ |     $$  __$$ | $$ |$$\ $$ |$$ |  $$ |$$ |  $$ | \____$$\
    $$$$$$$  |\$$$$$$$\ \$$$$$$$\ $$ |\$$$$$$$ |$$ |     \$$$$$$$ | \$$$$  |$$ |\$$$$$$  |$$ |  $$ |$$$$$$$  |
    \_______/  \_______| \_______|\__| \_______|\__|      \_______|  \____/ \__| \______/ \__|  \__|\_______/
     */

    // OpenCV Camera
    private UGContourRingPipeline pipeline; // This is the pipeline which was created for Ultimate Goal
    private OpenCvCamera camera; // The camera on the robot
    private int cMonitorViewId; // Variable to get identifier for camera.

    private Motor fL, fR, bL, bR;
    private Motor extraMotor;
    private SimpleServo simpleServo;

    private MecanumDrive mecDrive;  // Creates the "drivetrain" for the Roadrunner paths.
    private SampleMecanumDrive drive; // Mecanum Drive for Roadrunner Autonomous
    private Pose2d pStart = new Pose2d(-15.0, 3.0, Math.toRadians(-50.0)); // Bot will start (from center) 15 up, 3 to the left, and facing 50 degrees to the left from facing forward (on coord grid).
    private Trajectories trajs; // Class with all methods to run autonomous trajectories, will talk more about later.

    private ElapsedTime t0, t1;
    private VoltageSensor voltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         /$$$$$$           /$$   /$$     /$$           /$$ /$$                       /$$     /$$
        |_  $$_/          |__/  | $$    |__/          | $$|__/                      | $$    |__/
          | $$   /$$$$$$$  /$$ /$$$$$$   /$$  /$$$$$$ | $$ /$$ /$$$$$$$$  /$$$$$$  /$$$$$$   /$$  /$$$$$$  /$$$$$$$   /$$$$$$$
          | $$  | $$__  $$| $$|_  $$_/  | $$ |____  $$| $$| $$|____ /$$/ |____  $$|_  $$_/  | $$ /$$__  $$| $$__  $$ /$$_____/
          | $$  | $$  \ $$| $$  | $$    | $$  /$$$$$$$| $$| $$   /$$$$/   /$$$$$$$  | $$    | $$| $$  \ $$| $$  \ $$|  $$$$$$
          | $$  | $$  | $$| $$  | $$ /$$| $$ /$$__  $$| $$| $$  /$$__/   /$$__  $$  | $$ /$$| $$| $$  | $$| $$  | $$ \____  $$
         /$$$$$$| $$  | $$| $$  |  $$$$/| $$|  $$$$$$$| $$| $$ /$$$$$$$$|  $$$$$$$  |  $$$$/| $$|  $$$$$$/| $$  | $$ /$$$$$$$/
        |______/|__/  |__/|__/   \___/  |__/ \_______/|__/|__/|________/ \_______/   \___/  |__/ \______/ |__/  |__/|_______/
         */

        // Basics
        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");
        extraMotor = new Motor(hardwareMap, "extraMotor");
        simpleServo = new SimpleServo(hardwareMap, "simpleServo");
        mecDrive = new MecanumDrive(fL, fR, bL, bR);

        t0 = new ElapsedTime();
        t1 = new ElapsedTime();

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(pStart); // Sets the start pose for the MecanumDrivetrain to the pose initialized earlier.

        voltageSensor = hardwareMap.voltageSensor.iterator().next(); // How to get voltage
        double variableVolts = 12 / voltageSensor.getVoltage(); // Add comment later

        // Invert drivetrain motors.
        fL.setInverted(true);
        fR.setInverted(true);

        // Set modes for drivetrain
        fL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        ▒█▀▀█ █▀▀█ █▀▄▀█ █▀▀ █▀▀█ █▀▀█ 　 ▒█▀▀▀█ █▀▀ ▀▀█▀▀ █░░█ █▀▀█
        ▒█░░░ █▄▄█ █░▀░█ █▀▀ █▄▄▀ █▄▄█ 　 ░▀▀▀▄▄ █▀▀ ░░█░░ █░░█ █░░█
        ▒█▄▄█ ▀░░▀ ▀░░░▀ ▀▀▀ ▀░▀▀ ▀░░▀ 　 ▒█▄▄▄█ ▀▀▀ ░░▀░░ ░▀▀▀ █▀▀▀
         */

        // Gets Id of the view which has the camera feedback
        cMonitorViewId = this
            .hardwareMap
            .appContext
            .getResources().getIdentifier(
                    "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        // Gets the camera itself from hardware and puts it through OpenCv to create a usable version.
        camera = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cMonitorViewId);
        // Sets up pipeline for camera
        camera.setPipeline(pipeline = new UGContourRingPipeline(telemetry, true));
        // Opens the camera and sets its dimensions and orientation
        camera.openCameraDeviceAsync(() -> camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT));

        // Completes initialization
        waitForStart();

        /*
         _______                         __      __
        /       \                       /  |    /  |
        $$$$$$$  | __    __  _______   _$$ |_   $$/  _____  ____    ______
        $$ |__$$ |/  |  /  |/       \ / $$   |  /  |/     \/    \  /      \
        $$    $$< $$ |  $$ |$$$$$$$  |$$$$$$/   $$ |$$$$$$ $$$$  |/$$$$$$  |
        $$$$$$$  |$$ |  $$ |$$ |  $$ |  $$ | __ $$ |$$ | $$ | $$ |$$    $$ |
        $$ |  $$ |$$ \__$$ |$$ |  $$ |  $$ |/  |$$ |$$ | $$ | $$ |$$$$$$$$/
        $$ |  $$ |$$    $$/ $$ |  $$ |  $$  $$/ $$ |$$ | $$ | $$ |$$       |
        $$/   $$/  $$$$$$/  $$/   $$/    $$$$/  $$/ $$/  $$/  $$/  $$$$$$$/
         */

        // Set position for servos
        simpleServo.setPosition(0.5); // Sets position halfway

        // Set ouput for motors, generally 0
        fL.set(0);
        fR.set(0);
        bL.set(0);
        bR.set(0);
        extraMotor.set(0);

        // Reset time(rs)
        t0.reset();
        t1.reset();

        // Set up trajectories for later
        trajs = new Trajectories(drive, extraMotor, simpleServo, telemetry);

        // Start up the pipeline and get some telemetry displayed about it.
        UGContourRingPipeline.Height pHeight = pipeline.getHeight();
        telemetry.addData("Ring stack size: ", pHeight);
        telemetry.addData("Ring stack size: ", pHeight);

        // Updates telemetry each time.
        telemetry.update();

        /*
        ▒█▀▀█ █▀▀█ █▀▀█ █▀▀▄ █▀▀█ █░░█ █▀▀▄ █▀▀▄ █▀▀ █▀▀█
        ▒█▄▄▀ █░░█ █▄▄█ █░░█ █▄▄▀ █░░█ █░░█ █░░█ █▀▀ █▄▄▀
        ▒█░▒█ ▀▀▀▀ ▀░░▀ ▀▀▀░ ▀░▀▀ ░▀▀▀ ▀░░▀ ▀░░▀ ▀▀▀ ▀░▀▀
         */

        // Roadrunner allows for autonomous to be done through the use of coordinates and paths instead of guessing how much the motors needs to turn.
        // For this to be done, variables do have to be tuned in DriveConstants.java

        // Everything else in Trajectories.java

        // DISCLAIMER: The following auton is done with an Ultimate Goal pipeline, so some things will be different year-by-year.

        trajs.run(pHeight);
    }
}
