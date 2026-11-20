package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpMode", group = "TeleOp")
public class TeleOpMode extends OpMode {

    private Follower follower;

    Servo flowerServo;
    DcMotorEx intake1;
    DcMotorEx intake2;
    DcMotorEx shooter;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        flowerServo = hardwareMap.get(Servo.class, "flowerServo");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {
        shooter.setVelocity(-1500);
    }


    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );

        follower.manual(powers);
        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("velocity", shooter.getVelocity());
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        // Math.toDegrees() is a built-in java method

        if (gamepad1.a) {
            flowerServo.setPosition(0);
        }

        if (gamepad1.y) {
            flowerServo.setPosition(1);
        }

        if (gamepad1.right_trigger > 0) {
            intake1.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            intake1.setPower(-gamepad1.left_trigger);

        } else {

            if (gamepad1.right_bumper) {
                intake1.setPower(1);
                intake2.setPower(1);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }
        }

        if (gamepad1.x) {
            shooter.setVelocity(0);
        }

        if (gamepad1.b) {
            shooter.setVelocity(-1500);
        }
    }

}