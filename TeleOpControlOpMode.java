package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Single Driver (Rotation Fixed)", group="Teleop")
public class TeleOpControlOpMode extends OpMode {

    private final ElapsedTime macroTimer = new ElapsedTime();
    private enum MacroState { IDLE, STEP1_REVERSE, STEP2_WAIT, STEP3_FORWARD }
    private MacroState currentMacroState = MacroState.IDLE;
    private boolean lastMacroButton = false;

    // --- 参数 ---
    private static final double STRAFE_CORRECTION = 1.1;
    private static final double TURN_SENSITIVITY = 1.0;
    private static final double SPEED_MULTIPLIER = 1.0;
    private static final double SHOOTER_UP_RATIO = 0.65;

    private DcMotor frontLeftDrive = null;
    private DcMotor rearLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor indexerMotor = null;
    private DcMotor shooterDownMotor = null;
    private DcMotor shooterUpMotor = null;

    @Override
    public void init() {
        // 硬件映射
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rearLeftDrive   = hardwareMap.get(DcMotor.class, "rear_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        rearRightDrive  = hardwareMap.get(DcMotor.class, "rear_right");

        intakeMotor     = hardwareMap.get(DcMotor.class, "intake");
        indexerMotor    = hardwareMap.get(DcMotor.class, "indexer");
        shooterDownMotor = hardwareMap.get(DcMotor.class, "shooter_down");
        shooterUpMotor   = hardwareMap.get(DcMotor.class, "shooter_up");

        // --- 方向设置 (保持不变) ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // 功能电机
        shooterDownMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterUpMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        indexerMotor.setDirection(DcMotor.Direction.FORWARD);

        // 刹车
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        // 1. 输入处理 (保持之前修正后的逻辑)
        // 左摇杆X控制前后 (Axial)
        double axial   = gamepad1.left_stick_x;
        // 左摇杆Y控制平移 (Lateral)
        double lateral = -gamepad1.left_stick_y * STRAFE_CORRECTION;
        // 右摇杆X控制旋转
        double rx      = gamepad1.right_stick_x * TURN_SENSITIVITY;

        // =========================================================
        //            关键修改：旋转逻辑重组 (Fix Rotation)
        // =========================================================

        // 之前的逻辑是 FL+rx, FR-rx (左正右负)
        // 但因为你的车逻辑转了90度，前半部分(FL+FR)在打架。

        // 新逻辑：前半部分(FL+FR)同向，后半部分(RL+RR)反向
        // 这样它们就会形成一个巨大的旋转力矩，而不会互相抵消

        double frontLeftPower  = axial + lateral + rx; // FL 加旋转
        double frontRightPower = axial - lateral + rx; // FR 也要加旋转 (原来是减)

        double rearLeftPower   = axial - lateral - rx; // RL 减旋转 (原来是加)
        double rearRightPower  = axial + lateral - rx; // RR 减旋转

        // 如果发现旋转方向反了（推左变右转），把上面四个 rx 的正负号全部反过来即可：
        // FL - rx
        // FR - rx
        // RL + rx
        // RR + rx

        // =========================================================

        // 归一化
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(rearLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.abs(rearRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            rearLeftPower /= maxPower;
            frontRightPower /= maxPower;
            rearRightPower /= maxPower;
        }

        frontLeftDrive.setPower(frontLeftPower * SPEED_MULTIPLIER);
        rearLeftDrive.setPower(rearLeftPower * SPEED_MULTIPLIER);
        frontRightDrive.setPower(frontRightPower * SPEED_MULTIPLIER);
        rearRightDrive.setPower(rearRightPower * SPEED_MULTIPLIER);

        // --- Intake & Macro & Shooter (保持不变) ---
        double intakePower = 0.0;
        double indexerPower = 0.0;
        boolean manualIntakeActive = false;

        if (gamepad1.a) { intakePower = 1.0; manualIntakeActive = true; }
        else if (gamepad1.b) { intakePower = -1.0; manualIntakeActive = true; }
        if (gamepad1.x) { indexerPower = 1.0; }
        else if (gamepad1.y) { indexerPower = -1.0; }

        boolean currentMacroButton = gamepad1.right_bumper;
        if (manualIntakeActive) {
            currentMacroState = MacroState.IDLE;
        } else {
            if (currentMacroButton && !lastMacroButton && currentMacroState == MacroState.IDLE) {
                currentMacroState = MacroState.STEP1_REVERSE;
                macroTimer.reset();
            }
            switch (currentMacroState) {
                case STEP1_REVERSE:
                    intakePower = -1.0;
                    if (macroTimer.seconds() > 0.24) currentMacroState = MacroState.STEP2_WAIT;
                    break;
                case STEP2_WAIT:
                    intakePower = 0.0;
                    if (macroTimer.seconds() > 0.5) currentMacroState = MacroState.STEP3_FORWARD;
                    break;
                case STEP3_FORWARD:
                    intakePower = 1.0; indexerPower = -1.0;
                    if (macroTimer.seconds() > 1.5) currentMacroState = MacroState.IDLE;
                    break;
                case IDLE: break;
            }
        }
        lastMacroButton = currentMacroButton;
        intakeMotor.setPower(intakePower);
        indexerMotor.setPower(indexerPower);

        double triggerInput = gamepad1.right_trigger;
        shooterUpMotor.setPower(triggerInput * SHOOTER_UP_RATIO);
        shooterDownMotor.setPower(triggerInput);

        telemetry.addData("Mode", "Rotation Fixed (Front/Back Split)");
        telemetry.addData("Motors", "FL:%.2f FR:%.2f RL:%.2f RR:%.2f",
                frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
        telemetry.update();
    }
}