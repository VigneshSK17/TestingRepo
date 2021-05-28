package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {

    private MecanumDrive drive;
    private Motor fL, fR, bL, bR;
//    private RevIMU revIMU;
//    boolean fieldCentric;

    public DriveSubsystem(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        fL = frontLeft;
        fR = frontRight;
        bL = backLeft;
        bR = backRight;
//        fieldCentric = false;
        drive = new MecanumDrive(fL, fR, bL, bR);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed) {
//        if(!fieldCentric) {
//            drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, true);
//        }
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed, true);

    }

}
