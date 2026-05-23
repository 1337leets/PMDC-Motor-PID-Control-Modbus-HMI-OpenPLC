"""
Motor Control Bridge  (v3 - live PID tuning)
============================================
OpenPLC Modbus register mapping:

  OUTPUTS (HMI reads these):
    %QW0  -> Holding reg 0  = pwm_output
    %QW1  -> Holding reg 1  = display_rpm      (actual RPM, filtered)
    %QW2  -> Holding reg 2  = display_target   (ramped setpoint)
    %QW3  -> Holding reg 3  = display_pid_out  (pwm% x 100)
    %QX0.0-> Coil 0         = fault_led

  INPUTS (HMI writes these):
    %QW4  -> Holding reg 4  = setpoint_raw     (0-32767)
    %QX0.1-> Coil 1         = enable_btn
    %QW5  -> Holding reg 5  = Kp x 1000         (live tuning)
    %QW6  -> Holding reg 6  = Ki x 1000
    %QW7  -> Holding reg 7  = Kd x 1000
    %QW8  -> Holding reg 8  = Max Accel (RPM/s)

Run: python bridge.py
Then open motor_hmi_v3.html in browser.
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from pymodbus.client import ModbusTcpClient
import threading
import time

app = Flask(__name__)
CORS(app)

client = ModbusTcpClient(host="localhost", port=502)
client_lock = threading.Lock()

state = {
    "connected":   False,
    "rpm_actual":  0.0,
    "rpm_ramped":  0.0,
    "pwm_percent": 0.0,
    "fault":       False,
    "setpoint":    0,
    "enabled":     False,
    # live tuning readback
    "kp":          0.056,
    "ki":          0.111,
    "kd":          0.008,
    "accel":       500,
}

def connect():
    with client_lock:
        result = client.connect()
        state["connected"] = result
        if result:
            print("[bridge] Connected to OpenPLC on localhost:502")
        else:
            print("[bridge] Could not connect - is PLC running?")
    return result

def read_loop():
    while True:
        try:
            with client_lock:
                if not client.is_socket_open():
                    client.connect()

                # Read holding registers 1,2,3 -> display_rpm, display_target, display_pid_out
                rr = client.read_holding_registers(1, count=3)
                if not rr.isError():
                    state["rpm_actual"]  = float(rr.registers[0])
                    state["rpm_ramped"]  = float(rr.registers[1])
                    state["pwm_percent"] = rr.registers[2] / 100.0
                    state["connected"]   = True
                else:
                    print(f"[bridge] Register read error: {rr}")
                    state["connected"] = False

                # Read coil 0 -> fault_led
                rc = client.read_coils(0, count=1)
                if not rc.isError():
                    state["fault"] = bool(rc.bits[0])

                # Read back live tuning regs 5,6,7,8 (so HMI shows true PLC values)
                rp = client.read_holding_registers(5, count=4)
                if not rp.isError():
                    state["kp"]    = rp.registers[0] / 1000.0
                    state["ki"]    = rp.registers[1] / 1000.0
                    state["kd"]    = rp.registers[2] / 1000.0
                    state["accel"] = rp.registers[3]

        except Exception as e:
            state["connected"] = False
            print(f"[bridge] Read error: {e}")

        time.sleep(0.1)

@app.route("/state", methods=["GET"])
def get_state():
    return jsonify(state)

@app.route("/setpoint", methods=["POST"])
def set_setpoint():
    data = request.json
    pct  = float(data.get("percent", 0))
    raw  = int(pct / 100.0 * 32767)
    state["setpoint"] = raw
    try:
        with client_lock:
            result = client.write_register(4, raw)
            if result.isError():
                print(f"[bridge] Setpoint write error: {result}")
            else:
                print(f"[bridge] Setpoint -> reg4 = {raw}  ({pct:.0f}%)")
    except Exception as e:
        print(f"[bridge] Write setpoint error: {e}")
    return jsonify({"ok": True, "raw": raw})

@app.route("/enable", methods=["POST"])
def set_enable():
    data    = request.json
    enabled = bool(data.get("enabled", False))
    state["enabled"] = enabled
    try:
        with client_lock:
            result = client.write_coil(1, enabled)
            if result.isError():
                print(f"[bridge] Enable write error: {result}")
            else:
                print(f"[bridge] Enable -> coil1 = {enabled}")
    except Exception as e:
        print(f"[bridge] Write enable error: {e}")
    return jsonify({"ok": True, "enabled": enabled})

@app.route("/pid", methods=["POST"])
def set_pid():
    """Live PID tuning. Accepts any subset of kp, ki, kd, accel."""
    data = request.json or {}
    written = {}
    try:
        with client_lock:
            if "kp" in data:
                v = int(round(float(data["kp"]) * 1000))
                v = max(0, min(65535, v))
                client.write_register(5, v)
                written["kp"] = v
            if "ki" in data:
                v = int(round(float(data["ki"]) * 1000))
                v = max(0, min(65535, v))
                client.write_register(6, v)
                written["ki"] = v
            if "kd" in data:
                v = int(round(float(data["kd"]) * 1000))
                v = max(0, min(65535, v))
                client.write_register(7, v)
                written["kd"] = v
            if "accel" in data:
                v = int(round(float(data["accel"])))
                v = max(1, min(65535, v))
                client.write_register(8, v)
                written["accel"] = v
        print(f"[bridge] PID tuning -> {written}")
    except Exception as e:
        print(f"[bridge] Write PID error: {e}")
        return jsonify({"ok": False, "error": str(e)})
    return jsonify({"ok": True, "written": written})

if __name__ == "__main__":
    print("=" * 50)
    print("  Motor Control Bridge v3  (live PID tuning)")
    print("  OpenPLC Modbus -> localhost:502")
    print("  HMI Flask      -> localhost:5000")
    print("=" * 50)
    print("  Reading:  Holding reg 1,2,3  (RPM, Ramped, PWM%)")
    print("            Holding reg 5,6,7,8 (Kp,Ki,Kd,Accel readback)")
    print("  Writing:  Holding reg 4      (Setpoint)")
    print("            Coil 1             (Enable)")
    print("            Holding reg 5-8    (PID tuning)")
    print()
    connect()
    t = threading.Thread(target=read_loop, daemon=True)
    t.start()
    app.run(host="localhost", port=5000, debug=False)
