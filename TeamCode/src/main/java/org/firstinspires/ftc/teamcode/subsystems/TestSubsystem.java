package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class TestSubsystem extends SubsystemBase {

    private Servo testServo;

    public TestSubsystem(Servo theServo) {
        testServo = theServo;
    }

    public void grab() {
        testServo.setPosition(0.5);
    }

    public void release() {
        testServo.setPosition(0.0);
    }

}
