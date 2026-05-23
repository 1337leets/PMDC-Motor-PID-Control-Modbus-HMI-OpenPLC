PROGRAM motor_control
VAR
    (* === I/O bound to Modbus via AT keyword === *)
    enable_btn      AT %QX0.1 : BOOL := FALSE;
    setpoint_raw    AT %QW4   : INT  := 0;
    pwm_output      AT %QW0   : INT  := 0;
    display_rpm     AT %QW1   : INT  := 0;
    display_target  AT %QW2   : INT  := 0;
    display_pid_out AT %QW3   : INT  := 0;
    fault_led       AT %QX0.0 : BOOL := FALSE;

    (* === Live tuning inputs (HMI writes these, scaled x1000 except accel) === *)
    kp_raw          AT %QW5   : INT  := 56;
    ki_raw          AT %QW6   : INT  := 111;
    kd_raw          AT %QW7   : INT  := 8;
    accel_raw       AT %QW8   : INT  := 500;
END_VAR

VAR
    (* === Tunable parameters (now read from registers each scan) === *)
    MAX_RPM        : REAL := 3000.0;
    MAX_ACCEL      : REAL := 500.0;
    CYCLE_TIME_S   : REAL := 0.010;
    Kp             : REAL := 0.056;
    Ki             : REAL := 0.111;
    Kd             : REAL := 0.008;
    INTEGRAL_LIMIT : REAL := 1000.0;
    DEADBAND_RPM   : REAL := 3.0;
    FILTER_ALPHA   : REAL := 0.2;
    D_FILTER       : REAL := 0.1;

    (* === Motor model === *)
    motor_gain_sim : REAL := 30.0;
    motor_tau      : REAL := 0.5;

    (* === Process variables === *)
    rpm_actual     : REAL := 0.0;
    rpm_filtered   : REAL := 0.0;
    last_rpm       : REAL := 0.0;
    rpm_setpoint   : REAL := 0.0;
    rpm_ramped     : REAL := 0.0;
    max_step       : REAL := 0.0;
    ramp_delta     : REAL := 0.0;
    pid_error      : REAL := 0.0;
    pid_integral   : REAL := 0.0;
    pid_derivative : REAL := 0.0;
    d_raw          : REAL := 0.0;
    pid_output     : REAL := 0.0;
    pwm_percent    : REAL := 0.0;
    stall_timer    : TON;
    stall_fault    : BOOL := FALSE;
    heartbeat      : INT  := 0;
END_VAR

(* === HEARTBEAT === *)
heartbeat := heartbeat + 1;
IF heartbeat > 32000 THEN
    heartbeat := 0;
END_IF;

(* === BLOCK 0: Read live tuning params from registers === *)
(* HMI writes scaled integers; convert back to REAL. Guard against zero/garbage. *)
IF kp_raw >= 0 THEN
    Kp := INT_TO_REAL(kp_raw) / 1000.0;
END_IF;
IF ki_raw >= 0 THEN
    Ki := INT_TO_REAL(ki_raw) / 1000.0;
END_IF;
IF kd_raw >= 0 THEN
    Kd := INT_TO_REAL(kd_raw) / 1000.0;
END_IF;
IF accel_raw > 0 THEN
    MAX_ACCEL := INT_TO_REAL(accel_raw);
END_IF;

(* === BLOCK 1: Software motor model === *)
rpm_actual := rpm_actual + (CYCLE_TIME_S / motor_tau) *
              (motor_gain_sim * pwm_percent - rpm_actual);
IF rpm_actual < 0.0 THEN
    rpm_actual := 0.0;
END_IF;

(* === BLOCK 1b: Measurement filter (EMA) === *)
rpm_filtered := FILTER_ALPHA * rpm_actual + (1.0 - FILTER_ALPHA) * rpm_filtered;

(* === BLOCK 2: Setpoint scaling === *)
IF enable_btn THEN
    rpm_setpoint := INT_TO_REAL(setpoint_raw) / 32767.0 * MAX_RPM;
ELSE
    rpm_setpoint := 0.0;
END_IF;

(* === BLOCK 3: Ramp limiter === *)
max_step   := MAX_ACCEL * CYCLE_TIME_S;
ramp_delta := rpm_setpoint - rpm_ramped;
IF ramp_delta > max_step THEN
    rpm_ramped := rpm_ramped + max_step;
ELSIF ramp_delta < (-1.0 * max_step) THEN
    rpm_ramped := rpm_ramped - max_step;
ELSE
    rpm_ramped := rpm_setpoint;
END_IF;

(* === BLOCK 4: PID controller (filtered derivative, anti-windup) === *)
IF enable_btn THEN
    pid_error := rpm_ramped - rpm_filtered;

    (* Deadband: cok kucuk hatalarda integrali dondur *)
    IF (pid_error < DEADBAND_RPM) AND (pid_error > (-1.0 * DEADBAND_RPM)) THEN
        pid_error := 0.0;
    END_IF;

    (* Onceki ciktiyi hesapla (anti-windup karari icin) *)
    pid_output := Kp * pid_error + Ki * pid_integral + Kd * pid_derivative;

    (* Conditional integration: sadece PWM doygun degilken integrali artir *)
    IF (pid_output < 100.0) AND (pid_output > 0.0) THEN
        pid_integral := pid_integral + pid_error * CYCLE_TIME_S;
        IF pid_integral >  INTEGRAL_LIMIT THEN pid_integral :=  INTEGRAL_LIMIT; END_IF;
        IF pid_integral < (-1.0 * INTEGRAL_LIMIT) THEN pid_integral := (-1.0 * INTEGRAL_LIMIT); END_IF;
    END_IF;

    (* Filtered derivative on measurement *)
    d_raw := (-1.0) * (rpm_filtered - last_rpm) / CYCLE_TIME_S;
    pid_derivative := D_FILTER * d_raw + (1.0 - D_FILTER) * pid_derivative;
    last_rpm := rpm_filtered;

    (* Cikti tekrar hesapla (guncel integral ile) *)
    pid_output := Kp * pid_error + Ki * pid_integral + Kd * pid_derivative;

    IF    pid_output > 100.0 THEN pwm_percent := 100.0;
    ELSIF pid_output <   0.0 THEN pwm_percent :=   0.0;
    ELSE                          pwm_percent := pid_output;
    END_IF;
ELSE
    pwm_percent    := 0.0;
    pid_integral   := 0.0;
    pid_derivative := 0.0;
    d_raw          := 0.0;
    last_rpm       := rpm_filtered;
    rpm_ramped     := rpm_actual;
END_IF;

(* === BLOCK 5: Stall detection === *)
stall_timer(IN := (rpm_ramped > 100.0) AND (rpm_filtered < 50.0) AND enable_btn, PT := T#10s);
IF stall_timer.Q  THEN stall_fault := TRUE;  END_IF;
IF NOT enable_btn THEN stall_fault := FALSE; END_IF;
fault_led := stall_fault;
IF stall_fault THEN pwm_percent := 0.0; END_IF;

(* === BLOCK 6: Output mapping === *)
pwm_output      := REAL_TO_INT(pwm_percent / 100.0 * 32767.0);
display_rpm     := REAL_TO_INT(rpm_filtered);
display_target  := REAL_TO_INT(rpm_ramped);
display_pid_out := REAL_TO_INT(pwm_percent * 100.0);

END_PROGRAM

CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK MainTask(INTERVAL := T#10ms, PRIORITY := 0);
    PROGRAM MainInst WITH MainTask : motor_control;
  END_RESOURCE
END_CONFIGURATION
