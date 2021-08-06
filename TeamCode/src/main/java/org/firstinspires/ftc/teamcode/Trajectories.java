package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.TimedAction;

import java.util.Arrays;

public class Trajectories {

    // Stuff which is needed to create trajectories and execute stuff during the trajectories.
    SampleMecanumDrive drive; // Gathers, sets up, and runs trajectories
    Motor extraMotor; // Motor for example
    SimpleServo simpleServo; // Servo for example
    Telemetry telemetry; // Telemetry to phone

    // Poses are essentially coordinates on the field with an angle to denote how the robot should be rotated.
    // The X and Y coordinates are inverted from the normal coordinate plane (annoying but whatever).
    private Pose2d pStart = new Pose2d(-15.0, 3.0, Math.toRadians(-50.0)); // Bot will start (from center) 15 up, 3 to the left, and facing 50 degrees to the left from facing forward (on coord grid).
    private Pose2d p1 = new Pose2d(24.0, -33.0, Math.toRadians(0.0)); // First pose after start
    private Pose2d p2 = new Pose2d(39.0, 24.0, Math.toRadians(-90.0)); // Second pose after start
    private Pose2d p3 = new Pose2d(33.0, 54.0, Math.toRadians(90.0)); // Third pose after start

    // This is a basic constructor in Java. It allows for the creation of the Trajectories object for use by the Auton class
    // The variables inside the () are parameters, and these act as inputs, which can modify a program.
    public Trajectories(SampleMecanumDrive drive, Motor extraMotor, SimpleServo simpleServo, Telemetry telemetry) {
        this.drive = drive;
        this.extraMotor = extraMotor;
        this.simpleServo = simpleServo;
        this.telemetry = telemetry;
    }

    // This run method takes in pipeline height, and then picks which method to run based on that
    public void run(UGContourRingPipeline.Height pHeight) {

        telemetry.addData("Ring Stack Size: ", pHeight);

        if(pHeight == UGContourRingPipeline.Height.FOUR) {
            fourRing();
        } else if(pHeight == UGContourRingPipeline.Height.ONE) {
            oneRing();
        } else {
            zeroRing();
        }

    }

    /*
    These following 3 methods hold the actual trajectories.

    Trajectory: A set of points for the robot to pass through with velocity measured as well.
    A TrajectoryBuilder is made up of functions to facilitate motion and markers
        Motion Functions:
            - Allow for movement of bot from simple straight line to accurate curves (as shown here)
            - All types at https://www.learnroadrunner.com/trajectorybuilder-functions.html#trajectorybuilder-function-list
        Markers:
            - Allow for the robot to conduct and action between sections of a trajectory for many different reasons.
            - More info at https://www.learnroadrunner.com/markers.html#marker-guide

    .build() builds the Trajectory, therefore ending that trajectory.
    A trajectory is run by calling the followTrajectory method of the SimpleMecanumDrive with a parameter of the trajectory.

    Note: Each is a private method because I do not want any other class accessing them.
    Note 1: These are not the actual trajectories, these are just 1 example with various stuff mixed in.
     */

    // If 4 rings are detected, run this
    private void fourRing() {
        TimedAction tAMotorRev = new TimedAction(
                () -> extraMotor.set(1), // Starts by running this
                () -> extraMotor.set(0), // After 100 ms it runs this
                100,
                true
        );

        Trajectory traj = drive.trajectoryBuilder(pStart)
                .splineToSplineHeading(p1, Math.toRadians(-30.0))
                // How to make motor go to max speed for 100 seconds and then turn off using basic lambda
                .addDisplacementMarker(() -> {
                    extraMotor.set(1);
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        telemetry.addData(e.getMessage(), null);
                    }
                    extraMotor.set(0);
                })
                .splineToSplineHeading(p2, Math.toRadians(90.0))
                // How to do same thing as above with TimedAction helper class and some Lamba trickery.
                .addDisplacementMarker(tAMotorRev::run)
                .splineToSplineHeading(p3, Math.toRadians(90.0))
                .build();

        drive.followTrajectory(traj);

    }

    private void oneRing() {
        TimedAction tAServoTurn = new TimedAction(
                () -> simpleServo.setPosition(0.75),
                () -> extraMotor.set(0),
                100,
                true
        );

        Trajectory traj = drive.trajectoryBuilder(pStart)
                .splineToSplineHeading(p1, Math.toRadians(-30.0))
                .addDisplacementMarker(tAServoTurn::run)
                // This is how to adject the velocity (aka speed) of the bot
                .splineToSplineHeading(p2, Math.toRadians(90.0),
                        new MinVelocityConstraint(Arrays.asList(
                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                            new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH)
                        )),
                        new ProfileAccelerationConstraint(40))
                .splineToSplineHeading(p3, Math.toRadians(90.0))
                .build();

        drive.followTrajectory(traj);

    }

    private void zeroRing() {

        Trajectory traj = drive.trajectoryBuilder(pStart)
                .splineToSplineHeading(p1, Math.toRadians(-30.0))
                .splineToSplineHeading(p2, Math.toRadians(90.0))
                .splineToSplineHeading(p3, Math.toRadians(90.0))
                .build();

        drive.followTrajectory(traj);

        // Note for self: Can get previous trajectory end point with traj.end()
    }


}
