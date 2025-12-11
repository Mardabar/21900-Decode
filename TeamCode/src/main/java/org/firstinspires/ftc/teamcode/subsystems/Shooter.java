package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class Shooter {
    private final DcMotorEx rs;
    private final DcMotorEx ls;
    private final DcMotorEx elbow;


    public double kS = 0.004, kV = 0.00039, kP = 0.001;
    private VoltageSensor voltageSensor;


    // These are the three main positions for shooting
    public static double farVelocity;
    public static double midLineVelocity;
    public static double closeLineVelocity;
    public static double topBoxVelocity = 500;

    private double targetVelocity = 0;

    public TelemetryManager telemetryManager;


    public Shooter(HardwareMap hardwareMap){

        rs = hardwareMap.get(DcMotorEx.class, "rs");
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");


        MotorConfigurationType configR = rs.getMotorType().clone();
        configR.setAchieveableMaxRPMFraction(1.0);
        rs.setMotorType(configR);
        rs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorConfigurationType configL = ls.getMotorType().clone();
        configL.setAchieveableMaxRPMFraction(1.0);
        ls.setMotorType(configL);
        ls.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setElbowTarget(0);
    }

    public void update(Gamepad gamepad){
        updateMotors();
        updateControls(gamepad);
    }

    public void updateControls(Gamepad gamepad){


        if (gamepad.yWasPressed()){
            setVelocity(0);
        }

        if(gamepad.dpadUpWasPressed()) {
            setVelocity(topBoxVelocity);
            //setElbowTarget(520);
        }

        if(gamepad.right_trigger > 0.2) {
            setVelocity(rs.getVelocity() - 100);
        } else if(gamepad.left_trigger > 0.2) {
            setVelocity(rs.getVelocity() + 100);
        }
    }

    public void updateTelemetry(TelemetryManager telemetry) {
        telemetry.addData("Shooter Left Velocity", ls.getVelocity());
        telemetry.addData("Shooter Right Velocity", ls.getVelocity());
        telemetry.addData("Elbow pos", elbow.getCurrentPosition());
        telemetry.addData("Expansion Hub Voltage", voltageSensor.getVoltage());
        telemetry.addData("Shooter Left Target", targetVelocity);
        telemetry.addData("Shooter Right Target", -targetVelocity);
    }


    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angle);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void updateMotors(){
        double voltageCompenstation = voltageSensor.getVoltage() / 13.48;

        ls.setPower((((kV / voltageCompenstation) * targetVelocity) + (kP * (targetVelocity - rs.getVelocity())) + kS));
        rs.setPower(-1*(((kV / voltageCompenstation) * targetVelocity) + (kP * (targetVelocity - rs.getVelocity())) + kS));
    }



    public void setPower(double power) {
        rs.setPower(power);
        ls.setPower(-power);
    }

    public void setVelocity(double target) {
        targetVelocity = target;
    }


}
