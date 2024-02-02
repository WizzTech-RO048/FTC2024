package org.firstinspires.ftc.teamcode.Robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class Gripper {
    private final double GRAB_POSITION_1 = 0.5, GRAB_POSITION_2 = 1.0;
    private final double RELEASE_POSITION_1 = GRAB_POSITION_1+0.1;
    private final double RELEASE_POSITION_2 = GRAB_POSITION_2-0.1;

    private final double LEFT_PICKUP = 0.0, RIGHT_PICKUP = 1.0-0.13;
    private final double LEFT_RELEASE = 1.0-0.30, RIGHT_RELEASE = 0.3;
    private final double LEFT_DEFAULT = 0.5, RIGHT_DEFAULT = 0.5;

<<<<<<< HEAD
    private final double OPEN_BARIER_POS = 0.50, CLOSE_BARIER_POS = 0.55;
=======
    private final double OPEN_BARIER_POS = 0.48, CLOSE_BARIER_POS = 0.60;
>>>>>>> af994f3 (sper ca e autonomia buna)

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;

//    private final Servo left_gripper, right_gripper;
   // private final Servo gheara_right, gheara_left;
    private final Servo barier;
    private final CRServo intake;

    Gripper(@NonNull final Parameters parameters) {
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry object was not set");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set");

//        left_gripper = hardwareMap.get(Servo.class, "left_gripper");
//        right_gripper = hardwareMap.get(Servo.class, "right_gripper");

       // gheara_left = hardwareMap.get(Servo.class, "gheara_left");
       // gheara_right = hardwareMap.get(Servo.class, "gheara_right");
        barier = hardwareMap.get(Servo.class, "barier");

        intake = hardwareMap.get(CRServo.class, "crservo");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

//    public void pickPixels() {
//        gheara_left.setPosition(LEFT_PICKUP);
//        gheara_right.setPosition(RIGHT_PICKUP);
//    }
//
//    public void leavePixels() {
//        gheara_left.setPosition(LEFT_RELEASE);
//        gheara_right.setPosition(RIGHT_RELEASE);
//    }
//
//    public void defaultPickupPixelPos() {
//        gheara_left.setPosition(LEFT_DEFAULT);
//        gheara_right.setPosition(RIGHT_DEFAULT);
//    }

    public void closeBarier() {
        barier.setPosition(CLOSE_BARIER_POS);
    }

    public void openBarier() {
        barier.setPosition(OPEN_BARIER_POS);
    }

    public void rotateIntake(double speed) {
        intake.setPower(speed);
    }

//    public void grab() {
//        left_gripper.setPosition(GRAB_POSITION_1);
//        right_gripper.setPosition(GRAB_POSITION_2);
//    }
//
//    public void release() {
//        left_gripper.setPosition(RELEASE_POSITION_1);
//        right_gripper.setPosition(RELEASE_POSITION_2);
//    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
    }
}