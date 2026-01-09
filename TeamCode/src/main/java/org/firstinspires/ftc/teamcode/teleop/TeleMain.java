package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

@TeleOp(name = "TeleMain")
public class TeleMain extends LinearOpMode {
    private ShootSystem shooter;

    // Drive Vars

    private DcMotorEx lb;
    private DcMotorEx rb;
    private DcMotorEx lf;
    private DcMotorEx rf;

    private final double driveSpeed = 1;
    private final double acceleration = 4;
    private final double turnAccel = 3;

    private double lStickPosX;
    private double lStickPosY;
    private double rStickPosX;
    private final double stickClampMin = 0.3;
    private final double stickClampMax = 1;

    @Override
    public void runOpMode(){ // INITIALIZATION
        InitMotors();
        SetDriveDirection("forward");
        SetBrakes();
        shooter = new ShootSystem(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            Drive();

            if (gamepad2.a) shooter.Shoot();
            if (gamepad2.dpad_up) shooter.UseFeeder();
            if (gamepad2.aWasReleased()) shooter.StopMotors();
        }
    }

    private void Drive(){
        ClampSpeed();
        lb.setPower((rStickPosX * -driveSpeed) + (driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
        rb.setPower((rStickPosX * driveSpeed) + (-driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
        lf.setPower((rStickPosX * -driveSpeed) + (-driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
        rf.setPower((rStickPosX * driveSpeed) + (driveSpeed * lStickPosX) + (driveSpeed * lStickPosY));
    }

    private void ClampSpeed(){
        lStickPosX = Math.clamp(Math.abs(gamepad1.left_stick_x), stickClampMin, stickClampMax)
                * Math.signum(gamepad1.left_stick_x);
        lStickPosY = Math.clamp(Math.abs(gamepad1.left_stick_y), stickClampMin, stickClampMax)
                * Math.signum(gamepad1.left_stick_y);
        SmoothSpeed(lStickPosX, lStickPosY, acceleration, turnAccel);
    }

    private void SmoothSpeed(double posX, double posY, double accelExp, double turnExp){
        lStickPosX = Math.pow(posX, accelExp);
        lStickPosY = Math.pow(posY, accelExp);
        rStickPosX = Math.pow(gamepad1.right_stick_x, turnExp);
    }

    private void InitMotors(){
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
    }

    private void SetDriveDirection(String direction){
        if (direction.equals("forward")){
            lb.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.REVERSE);
            lf.setDirection(DcMotorEx.Direction.FORWARD);
            rf.setDirection(DcMotorEx.Direction.REVERSE);
            return;
        }

        lb.setDirection(DcMotorEx.Direction.REVERSE);
        rb.setDirection(DcMotorEx.Direction.FORWARD);
        lf.setDirection(DcMotorEx.Direction.REVERSE);
        rf.setDirection(DcMotorEx.Direction.FORWARD);
    }

    private void SetBrakes(){
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
