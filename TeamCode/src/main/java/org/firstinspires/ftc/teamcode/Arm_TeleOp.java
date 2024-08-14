package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@Config //allows modifications in real time
@TeleOp (name = "ArmControl")

public class Arm_TeleOp extends OpMode {
    private PIDController controller;

    public static double p = 0.003, i = 0.03, d= 0;
    public static double f = 0.05;

    public static int target = 0; //target in ticks for motor mm
    private final double ticks_in_degree = 1425.1 / 360.0; //depends on gear ratio and encoder
    private DcMotorEx armMotor;

    private CRServo Intake1;
    private CRServo Intake2;


    @Override
    public void init() {
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "motor1");
        Intake1 = hardwareMap.get(CRServo.class, "crservo1");
        Intake2 = hardwareMap.get(CRServo.class, "crservo2");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    boolean load = false;
    boolean p_rt = false;
    boolean rt = false;
    int toggle = 0;

    @Override
    public void loop() {
        controller.setPID(p,i,d);
        int armPos = armMotor.getCurrentPosition();
        double PID = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(target / ticks_in_degree)) * f;
        double power = PID + ff;
        rt = gamepad1.right_bumper;
        if (gamepad1.left_bumper){
        target = 0;
        }
        if (gamepad1.right_bumper && p_rt != rt){
            if (toggle == 0) {
                // Load Pixels
                toggle += 1;
                target = 0;
                Intake1.setPower(1);
                Intake2.setPower(1);

            }
            else if (toggle == 1){
                // Travel with pixels
                toggle += 1;
                target = 500;
                Intake1.setPower(0);
                Intake2.setPower(0);
            }
            else if (toggle == 2){
                toggle += 1;
                target = 1700;
            }
            else if (toggle == 3){
                toggle = 0;
                target = 500;
            }
        }
        p_rt = rt;

        armMotor.setPower(power);

        telemetry.addData("pos ", armPos); //telemetry for tuning
        telemetry.addData("target ", target);
        telemetry.update();

    }
}
