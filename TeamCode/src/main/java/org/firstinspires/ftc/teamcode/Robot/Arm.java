package org.firstinspires.ftc.teamcode.Robot;

import android.os.Build;
import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

@RequiresApi(api = Build.VERSION_CODES.N)
public class Arm {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final ScheduledExecutorService scheduler;

    private final DcMotorEx arm;
    private final Servo gripper_rotation_left, gripper_rotation_right;

    private final double LEFT_INITIAL_POS = 0.0, RIGHT_INITIAL_POS = 0.97;
    private final double LEFT_RELEASE_POS = 0.97-0.45, RIGHT_RELEASE_POS = 0.0+0.45;

    Arm(@NonNull final Parameters parameters) {
        scheduler = Objects.requireNonNull(parameters.scheduler, "Scheduler was not set");
        telemetry = Objects.requireNonNull(parameters.telemetry, "Telemetry was not set up");
        hardwareMap = Objects.requireNonNull(parameters.hardwareMap, "HardwareMap was not set up");

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gripper_rotation_left = hardwareMap.get(Servo.class, "gripper_rotation_left");
        gripper_rotation_right = hardwareMap.get(Servo.class, "gripper_rotation_right");
    }

    private ScheduledFuture<?> raiseArm = null;

    public ScheduledFuture<?> raiseArm(int targetPositionValue, double raisePower) {
        if (!Utils.isDone(raiseArm) && !raiseArm.cancel(true)) {
            return null;
        }

//        int targetPosition = (int) Math.floor(Utils.interpolate(0, armRaisedPosition, positionPercentage, 1));
        int initialPosition = arm.getCurrentPosition();

        if (targetPositionValue == initialPosition) {
            return null;
        }

        arm.setTargetPosition(targetPositionValue);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(targetPositionValue > initialPosition ? raisePower : -raisePower);

        raiseArm = Utils.poll(scheduler, () -> !arm.isBusy(), () -> arm.setPower(0), 10, TimeUnit.MILLISECONDS);

        return raiseArm;
    }


    public int getCurrentPositionArm() {
        return arm.getCurrentPosition();
    }

    public void gripperInitialPos() {
        gripper_rotation_left.setPosition(LEFT_INITIAL_POS);
        gripper_rotation_right.setPosition(RIGHT_INITIAL_POS);
    }

    public void gripperReleasePos() {
        gripper_rotation_left.setPosition(LEFT_RELEASE_POS);
        gripper_rotation_right.setPosition(RIGHT_RELEASE_POS);
    }

    public void stopArm() {
        // ----- stopping the slider moving -----
        arm.setPower(0.0);
    }

    public static class Parameters {
        public HardwareMap hardwareMap;
        public Telemetry telemetry;
        public ScheduledExecutorService scheduler;
    }

}