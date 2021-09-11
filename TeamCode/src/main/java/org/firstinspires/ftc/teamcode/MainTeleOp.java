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

@TeleOp(name = "Template TeleOp") // Names the OP mode in the phone menu
public class MainTeleOp extends LinearOpMode {

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

    // Chassis Constants
    // Used for odometry, which we use only in autonomous
    // public static final double TRACKWIDTH = 8.8;
    // public static final double WHEEL_DIAMETER = 1.42;
    // public static final double TICKS_PER_REV = 8192;
    // public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private Motor frontLeft, frontRight, backLeft, backRight; // Drivetrain Motors
    private Motor mExtra;
    private SimpleServo sSimple; // Servos
    // private RevIMU imu; // Measurement unit on the Rev Hub, not necessary with odom wheels
    private GamepadEx gPad1, gPad2; // Gamepads
    private MecanumDrive drivetrain; // The Drivetrain
    // private Motor.Encoder leftOdometer, rightOdometer, centerOdometer; // Odometer Wheels, not used here
    // private OdometrySubsystem odometry; // Odometry as a whole, not used here
    private ToggleButtonReader buttonReaderY1, buttonReaderA1, buttonReaderX1, buttonReaderB1, buttonReaderdPadUp1, buttonReaderdPadDown1, buttonReaderdPadRight1, buttonReaderdPadLeft1, buttonReaderBumperLeft1, buttonReaderBumperRight1, buttonReaderJoystickLeft1, buttonReaderJoystickRight1; // Toggle Buttons on Gamepad 1
    private ToggleButtonReader buttonReaderY2, buttonReaderA2, buttonReaderX2, buttonReaderB2, buttonReaderdPadUp2, buttonReaderdPadDown2, buttonReaderdPadRight2, buttonReaderdPadLeft2, buttonReaderBumperLeft2, buttonReaderBumperRight2, buttonReaderJoystickLeft2, buttonReaderJoystickRight2; // Toggle Buttons on Gamepad 2
    private ButtonReader rightTrigger1, leftTrigger1; // Triggers of controller
    private ButtonReader rightTrigger2, leftTrigger2; // Triggers of controller
    private VoltageSensor voltageSensor;

    private ElapsedTime t0;
    private TimedAction tA0; // Action for X amount of time

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

        // Initialize Drivetrain Motors
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        mExtra = new Motor(hardwareMap, "extra");

        // Initialize servos
        sSimple = new SimpleServo(hardwareMap, "servo", 0, 270);

        // Initialize drivetrain
        drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

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
        leftTrigger1 = new ButtonReader(gPad1, GamepadKeys.Button.LEFT_BUMPER);
        leftTrigger2 = new ButtonReader(gPad2, GamepadKeys.Button.LEFT_BUMPER);
        rightTrigger1 = new ButtonReader(gPad1, GamepadKeys.Button.RIGHT_BUMPER);
        rightTrigger2 = new ButtonReader(gPad2, GamepadKeys.Button.RIGHT_BUMPER);

        buttonReaderBumperLeft1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.LEFT_BUMPER);
        buttonReaderBumperLeft2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.LEFT_BUMPER);
        buttonReaderBumperRight1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.RIGHT_BUMPER);
        buttonReaderBumperRight2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.RIGHT_BUMPER);

        buttonReaderdPadUp1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_UP);
        buttonReaderdPadUp2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_UP);
        buttonReaderdPadDown1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_DOWN);
        buttonReaderdPadDown2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_DOWN);
        buttonReaderdPadLeft1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_LEFT);
        buttonReaderdPadLeft2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_LEFT);
        buttonReaderdPadRight1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.DPAD_RIGHT);
        buttonReaderdPadRight2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.DPAD_RIGHT);

        buttonReaderJoystickLeft1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        buttonReaderJoystickLeft2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        buttonReaderJoystickRight1 = new ToggleButtonReader(gPad1, GamepadKeys.Button.LEFT_STICK_BUTTON);
        buttonReaderJoystickRight2 = new ToggleButtonReader(gPad2, GamepadKeys.Button.LEFT_STICK_BUTTON);


        // Initialize Voltage Sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize ElapsedTime thing
        t0 = new ElapsedTime();

        /*
        ▒█▀▀▀ █░█ ▀▀█▀▀ █▀▀█ █▀▀█ 　 ▒█▀▀▀█ █▀▀ ▀▀█▀▀ █░░█ █▀▀█
        ▒█▀▀▀ ▄▀▄ ░░█░░ █▄▄▀ █▄▄█ 　 ░▀▀▀▄▄ █▀▀ ░░█░░ █░░█ █░░█
        ▒█▄▄▄ ▀░▀ ░░▀░░ ▀░▀▀ ▀░░▀ 　 ▒█▄▄▄█ ▀▀▀ ░░▀░░ ░▀▀▀ █▀▀▀
         */

        // Disable encoders on drivetrain motors
        frontLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Would set coefficients and stuff like that for motors here.
        mExtra.setInverted(true); // Inverts motor
        mExtra.resetEncoder(); // Resets encoder to default value, necessary prior to starting robot.

        // How to set up PID
        // More info: https://acme-robotics.gitbook.io/road-runner/tour/pid-control
        mExtra.setRunMode(Motor.RunMode.VelocityControl); // Enable velocity control
        mExtra.setVeloCoefficients(0.3,0,0); // KP: Proportional Gain, KI: Integral Gain, KD = Derivative Gain
        mExtra.setFeedforwardCoefficients(1, 1.305);

        // Create TimedAction, for example:
        tA0 = new TimedAction(
                () -> sSimple.setPosition(0.5), // Runs at start
                () -> sSimple.setPosition(0), // Runs at end
                120, // How many seconds until end
                true
        );

        // Now that initalization setup is done, wait for start button to be pressed
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

        t0.reset(); // Sets time to zero

        double speedMultiplier = 1;
        boolean movingBack = false;

        // This runs in a loop while the robot is running in TeleOp and hasn't been stopped.
        while(opModeIsActive() && !isStopRequested()) {

            // If left trigger is pressed to 25% or greater, drivetrain speed will be divided by 3.
            if(gamepad1.left_trigger >= 0.25) {
                speedMultiplier = 0.33;
            // If right trigger is pressed to 25% or greater, drivetrain speed will be set to max
            } else if (gamepad1.right_trigger >= 0.25) {
                speedMultiplier = 1;
            // Else, set drivetrain speed to 0.9
            } else {
                speedMultiplier = 0.9;
            }

            // If the right bumper of either controller is pressed and the timedAction is running already, reset the timedAction
            if((rightTrigger1.isDown() && !tA0.running()) || (rightTrigger2.isDown() && !tA0.running()))
                tA0.reset();

            // Always starts up the timedAction to possible be run.
            tA0.run();

            drivetrain.driveRobotCentric(-gPad1.getLeftX() * speedMultiplier, -gPad1.getLeftY() * speedMultiplier, -gPad1.getRightX() * speedMultiplier);

            // Voltage Sensor
            double v = voltageSensor.getVoltage(); // Gets voltage of battery
            double vMotorSpeed = Math.sqrt(10/v); // Takes the desired motor speed (10 in this case), and varies it based on how much voltage is in the battery.

            // Takes the status of each button (whether it is being pressed or not)
            buttonReaderBumperLeft1.readValue();
            buttonReaderBumperLeft2.readValue();
            buttonReaderBumperRight1.readValue();
            buttonReaderBumperRight2.readValue();

            buttonReaderA1.readValue();
            buttonReaderA2.readValue();
            buttonReaderB1.readValue();
            buttonReaderB2.readValue();
            buttonReaderX1.readValue();
            buttonReaderX2.readValue();
            buttonReaderY1.readValue();
            buttonReaderY2.readValue();

            buttonReaderdPadUp1.readValue();
            buttonReaderdPadUp2.readValue();
            buttonReaderdPadDown1.readValue();
            buttonReaderdPadDown2.readValue();
            buttonReaderdPadLeft1.readValue();
            buttonReaderdPadLeft2.readValue();
            buttonReaderdPadRight1.readValue();
            buttonReaderdPadRight2.readValue();

            buttonReaderJoystickLeft1.readValue();
            buttonReaderJoystickLeft2.readValue();
            buttonReaderJoystickRight1.readValue();
            buttonReaderJoystickRight2.readValue();

            // getState() on a buttonReader returns either true or false.

            // Checking if the A button is pressed either on controller 1 or 2.
            if(buttonReaderA1.getState() || buttonReaderA2.getState()) {
                // Sets the extra motor to half speed, then revert after 100 ms.
                mExtra.set(0.5);
                Thread.sleep(100);
                mExtra.stopMotor();
            }

            // Checking if right bumper was just pressed, then if right bumper was just released on either controller.
            // Both methods use a means of pausing, and then doing another action. Using TimedAction is preferred.
            if(rightTrigger1.wasJustPressed() || rightTrigger2.wasJustPressed()) {
                // Turns servo half way from minimum to maximum angle, wait 500 milliseconds, move the servo to the maximum angle
                sSimple.setPosition(0.5);
                Thread.sleep(500);
                sSimple.setPosition(1);

            } else if(rightTrigger1.wasJustReleased() || rightTrigger2.wasJustReleased()) {
                // Turns servo back to halfway, and then to starting position after 120 millisec delay.
                tA0.run();
            }

            // telemetry examples
            // this prints onto the phone stats about the running robot, useful for debugging and diagnosing issues.
            telemetry.addData("simpleservo position", sSimple.getPosition());
            telemetry.addData("simpleservo angle", sSimple.getAngle());
            telemetry.update(); // needed at end for telemetry to show up



        }


    }
}
