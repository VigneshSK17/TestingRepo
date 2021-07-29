package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.TimedAction;

@TeleOp(name = "TestTeleOp") // Names the
public class MainTeleOp extends LinearOpMode {

    /*
    ██████╗░███████╗░█████╗░██╗░░░░░░█████╗░██████╗░░█████╗░████████╗██╗░█████╗░███╗░░██╗░██████╗
    ██╔══██╗██╔════╝██╔══██╗██║░░░░░██╔══██╗██╔══██╗██╔══██╗╚══██╔══╝██║██╔══██╗████╗░██║██╔════╝
    ██║░░██║█████╗░░██║░░╚═╝██║░░░░░███████║██████╔╝███████║░░░██║░░░██║██║░░██║██╔██╗██║╚█████╗░
    ██║░░██║██╔══╝░░██║░░██╗██║░░░░░██╔══██║██╔══██╗██╔══██║░░░██║░░░██║██║░░██║██║╚████║░╚═══██╗
    ██████╔╝███████╗╚█████╔╝███████╗██║░░██║██║░░██║██║░░██║░░░██║░░░██║╚█████╔╝██║░╚███║██████╔╝
    ╚═════╝░╚══════╝░╚════╝░╚══════╝╚═╝░░╚═╝╚═╝░░╚═╝╚═╝░░╚═╝░░░╚═╝░░░╚═╝░╚════╝░╚═╝░░╚══╝╚═════╝░
     */

    // Chassis Constants
    // Used for odometry, which we use only in autonomous
    // public static final double TRACKWIDTH = 8.8;
    // public static final double WHEEL_DIAMETER = 1.42;
    // public static final double TICKS_PER_REV = 8192;
    // public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    // Initialize Variables
    private Motor fL, fR, bL, bR; // Drivetrain Motors
    private Motor extraMotor;
    private SimpleServo simpleServo; // Servos
    // private RevIMU imu; // Measurement unit on the Rev Hub, not necessary with odom wheels
    private GamepadEx gPad1, gPad2; // Gamepads
    private MecanumDrive drivetrain; // The Drivetrain
    // private Motor.Encoder leftOdometer, rightOdometer, centerOdometer; // Odometer Wheels, not used here
    // private OdometrySubsystem odometry; // Odometry as a whole, not used here
    private ToggleButtonReader buttonReaderY1, buttonReaderA1, buttonReaderX1, buttonReaderB1, buttonReaderdPadUp1, buttonReaderdPadDown1, buttonReaderdPadRight1, buttonReaderdPadLeft1; // Toggle Buttons on Gamepad 1
    private ToggleButtonReader buttonReaderY2, buttonReaderA2, buttonReaderX2, buttonReaderB2, buttonReaderdPadUp2, buttonReaderdPadDown2, buttonReaderdPadRight2, buttonReaderdPadLeft2; // Toggle Buttons on Gamepad 2
    private ButtonReader rightBumper1, leftBumper1; // Bumpers of controller
    private ButtonReader rightBumper2, leftBumper2; // Bumpers of controller
    private VoltageSensor voltageSensor;

    private ElapsedTime time;
    private TimedAction timedAction; // Action for X amount of time

    @Override
    public void runOpMode() throws InterruptedException {

        /*
       ██╗███╗░░██╗██╗████████╗██╗░█████╗░██╗░░░░░██╗███████╗░█████╗░████████╗██╗░█████╗░███╗░░██╗░██████╗
       ██║████╗░██║██║╚══██╔══╝██║██╔══██╗██║░░░░░██║╚════██║██╔══██╗╚══██╔══╝██║██╔══██╗████╗░██║██╔════╝
       ██║██╔██╗██║██║░░░██║░░░██║███████║██║░░░░░██║░░███╔═╝███████║░░░██║░░░██║██║░░██║██╔██╗██║╚█████╗░
       ██║██║╚████║██║░░░██║░░░██║██╔══██║██║░░░░░██║██╔══╝░░██╔══██║░░░██║░░░██║██║░░██║██║╚████║░╚═══██╗
       ██║██║░╚███║██║░░░██║░░░██║██║░░██║███████╗██║███████╗██║░░██║░░░██║░░░██║╚█████╔╝██║░╚███║██████╔╝
       ╚═╝╚═╝░░╚══╝╚═╝░░░╚═╝░░░╚═╝╚═╝░░╚═╝╚══════╝╚═╝╚══════╝╚═╝░░╚═╝░░░╚═╝░░░╚═╝░╚════╝░╚═╝░░╚══╝╚═════╝░
         */

        // Initialize Drivetrain Motors
        fL = new Motor(hardwareMap, "frontLeft");
        fR = new Motor(hardwareMap, "frontRight");
        bL = new Motor(hardwareMap, "backLeft");
        bR = new Motor(hardwareMap, "backRight");

        extraMotor = new Motor(hardwareMap, "extra");

        // Initialize servos
        simpleServo = new SimpleServo(hardwareMap, "servo", 0, 270);

        // Initialize drivetrain
        drivetrain = new MecanumDrive(fL, fR, bL, bR);

        // Initialize Gamepads
        gPad1 = new GamepadEx(gamepad1); gPad2 = new GamepadEx(gamepad2);

        // Initialize Gamepad Buttons
        buttonReaderX1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.X);
        buttonReaderX2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.X);
        buttonReaderY1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.Y);
        buttonReaderY2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.Y);
        buttonReaderA1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.A);
        buttonReaderA2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.A);
        buttonReaderB1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.B);
        buttonReaderB2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.B);

        // Initialize Other Gamepad Buttons
        leftBumper1 = new ButtonReader(gPad1, GamepadKeys.Button.LEFT_BUMPER);
        leftBumper2 = new ButtonReader(gPad2, GamepadKeys.Button.LEFT_BUMPER);
        rightBumper1 = new ButtonReader(gPad1, GamepadKeys.Button.RIGHT_BUMPER);
        rightBumper2 = new ButtonReader(gPad2, GamepadKeys.Button.RIGHT_BUMPER);

        buttonReaderdPadUp1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_UP);
        buttonReaderdPadUp2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_UP);
        buttonReaderdPadDown1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_DOWN);
        buttonReaderdPadDown2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_DOWN);
        buttonReaderdPadLeft1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_LEFT);
        buttonReaderdPadLeft2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_LEFT);
        buttonReaderdPadRight1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_RIGHT);
        buttonReaderdPadRight2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_RIGHT);

        // Initialize Voltage Sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize ElapsedTime thing
        time = new ElapsedTime();

        /*
        ▒█▀▀▀ █░█ ▀▀█▀▀ █▀▀█ █▀▀█ 　 ▒█▀▀▀█ █▀▀ ▀▀█▀▀ █░░█ █▀▀█
        ▒█▀▀▀ ▄▀▄ ░░█░░ █▄▄▀ █▄▄█ 　 ░▀▀▀▄▄ █▀▀ ░░█░░ █░░█ █░░█
        ▒█▄▄▄ ▀░▀ ░░▀░░ ▀░▀▀ ▀░░▀ 　 ▒█▄▄▄█ ▀▀▀ ░░▀░░ ░▀▀▀ █▀▀▀
         */

        // Disable encoders on drivetrain motors
        fL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bL.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Would set coefficients and stuff like that for motors here.
        extraMotor.setInverted(true); // Inverts motor
        extraMotor.resetEncoder(); // Resets encoder to default value, necessary prior to starting robot.

        // How to set up PID
        // More info: https://acme-robotics.gitbook.io/road-runner/tour/pid-control
        extraMotor.setRunMode(Motor.RunMode.VelocityControl); // Enable velocity control
        extraMotor.setVeloCoefficients(0.3,0,0); // KP: Proportional Gain, KI: Integral Gain, KD = Derivative Gain
        extraMotor.setFeedforwardCoefficients(1, 1.305);


        // Create TimedAction, for example:
        timedAction = new TimedAction(
                () -> simpleServo.setPosition(0.5), // Runs at start
                () -> simpleServo.setPosition(0), // Runs at end
                120, // How many seconds until end
                true
        );

        // Now that initalization setup is done, wait for start button to be pressed
        waitForStart();

        /*
        ██████╗░██╗░░░██╗███╗░░██╗████████╗██╗███╗░░░███╗███████╗
        ██╔══██╗██║░░░██║████╗░██║╚══██╔══╝██║████╗░████║██╔════╝
        ██████╔╝██║░░░██║██╔██╗██║░░░██║░░░██║██╔████╔██║█████╗░░
        ██╔══██╗██║░░░██║██║╚████║░░░██║░░░██║██║╚██╔╝██║██╔══╝░░
        ██║░░██║╚██████╔╝██║░╚███║░░░██║░░░██║██║░╚═╝░██║███████╗
        ╚═╝░░╚═╝░╚═════╝░╚═╝░░╚══╝░░░╚═╝░░░╚═╝╚═╝░░░░░╚═╝╚══════╝
         */

        time.reset(); // Sets time to zero

        double speedMultiplier = 1;
        boolean movingBack = false;

        // This runs in a loop while the robot is running in TeleOp and hasn't been stopped.
        while(opModeIsActive() && !isStopRequested()) {

            // If right bumper is pressed, drivetrain speed will be divided by 3.
            if(gamepad1.right_bumper) {
                speedMultiplier = 0.33;
            } else {
                speedMultiplier = 1;
            }

            // If the right bumper of either controller is pressed and the timedAction is running already, reset the timedAction
            if((rightBumper1.isDown() && !timedAction.running()) || (rightBumper2.isDown() && !timedAction.running()))
                timedAction.reset();

            // Always starts up the timedAction to possible be run.
            timedAction.run();

            drivetrain.driveRobotCentric(-gPad1.getLeftX() * speedMultiplier, -gPad1.getLeftY() * speedMultiplier, -gPad1.getRightX() * speedMultiplier);

            // Voltage Sensor
            double voltage = voltageSensor.getVoltage(); // Gets voltage of battery
            double motorSpeed = Math.sqrt(10/voltage); // Takes the desired motor speed (10 in this case), and varies it based on how much voltage is in the battery.

        }


    }
}