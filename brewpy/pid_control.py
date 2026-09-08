# pid_control.py
import time
import math
from typing import Optional, Tuple
from simple_pid import PID



class CascadePIDController:
    """
    Generic IO-free cascade PID controller.

    - Outer loop (master) controls the primary process variable (outer PV).
      Its output is the setpoint for the inner loop (inner SP).

    - Inner loop (slave) controls the secondary process variable (inner PV),
      and produces the final actuator output (e.g., % power).

    Usage pattern:
        controller = CascadePIDController()
        # configure gains, limits, modes, etc. as needed
        u, inner_sp = controller.compute(outer_pv=<float>, inner_pv=<float>)
        # write 'u' to your actuator elsewhere in your code

    The controller is IO-free: it does not read sensors nor write outputs.
    You supply PVs to compute() and use the returned output as you wish.

    The update runs on an internal cadence (pid_period_s). You can call compute()
    every scan; it will compute at most once per period and otherwise return
    the latest (u, inner_sp).
    """

    def __init__(self):
        # ===== Timing =====
        self.pid_period_s: float = 1.0   # Default 1 Hz (good match for slow sensors like DS18B20)
        now = time.monotonic()
        self._next_tick: float = now + self.pid_period_s

        # ===== PV filters (first-order low-pass) =====
        self.alpha_outer: float = 0.1
        self.alpha_inner: float = 0.1
        self._outer_pv_filt: Optional[float] = None
        self._inner_pv_filt: Optional[float] = None
        self._outer_pv_valid = False
        self._inner_pv_valid = False

        # ===== Outer loop config =====
        # User-commanded outer SP is rate-limited (ramped) before feeding the outer PID.
        self.outer_sp_cmd: float = 0.0         # engineering units of outer PV
        self.outer_sp_ramped: float = 0.0
        self.outer_sp_ramp_rate_per_min: float = 50.0  # units/min of outer PV

        # The outer PID output becomes the inner setpoint; clamp it to safe range.
        self.inner_sp_min: float = 0.0
        self.inner_sp_max: float = 100.0
        self.outer_output_min: float = 30.0
        self.outer_output_max: float = 85.0
        self.inner_i_min: float = 0
        self.inner_i_max: float = 0
        self.inner_i_clamp: bool = False

        self.pid_outer = PID(Kp=1.0, Ki=0.01, Kd=0.0, setpoint=self.outer_sp_ramped)
        self.pid_outer.sample_time = self.pid_period_s
        self.pid_outer.output_limits = (self.inner_sp_min, self.inner_sp_max)
        self.pid_outer.proportional_on_measurement = False  # typical for many slow processes

        # ===== Inner loop config =====
        self.pid_inner = PID(Kp=10.0, Ki=0.05, Kd=0.0, setpoint=0.0)
        self.pid_inner.sample_time = self.pid_period_s
        self.pid_inner.output_limits = (0.0, 100.0)  # e.g., 0–100% actuator
        self.pid_inner.proportional_on_measurement = False

        # ===== Modes / manual values =====
        self.outer_auto: bool = False
        self.inner_auto: bool = False
        self._manual_outer_output_as_inner_sp: float = 0.0   # inner SP when outer=MANUAL
        self._manual_inner_output: float = 0.0               # actuator output when inner=MANUAL
        self._manual_inner_setpoint: float = 0.0             # inner setpoint when outer=MANUAL (direct control just inner loop)

        # ===== Live diagnostics =====
        self.inner_sp: float = self.pid_inner.setpoint       # cascaded setpoint from outer
        self.actuator_output: float = 0.0                    # final output from inner

    # ---------- Utilities ----------
    @staticmethod
    def _low_pass(prev: Optional[float], new: float, alpha: float) -> float:
        if prev is None or (isinstance(prev, float) and math.isnan(prev)):
            return new
        return prev + alpha * (new - prev)

    @staticmethod
    def _clamp(val: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, val))

    def _update_ramped_outer_sp(self):
        """
        Rate-limit outer_sp_cmd -> outer_sp_ramped at outer_sp_ramp_rate_per_min.
        """
        rate_per_s = self.outer_sp_ramp_rate_per_min / 60.0
        delta_max = rate_per_s * self.pid_period_s
        tgt = self.outer_sp_cmd
        cur = self.outer_sp_ramped
        diff = tgt - cur


        if self._outer_pv_filt is None or abs(diff) > 5.0:
                self.outer_sp_ramped = tgt
                return


        if abs(diff) <= delta_max:
            self.outer_sp_ramped = tgt
        else:
            self.outer_sp_ramped = cur + math.copysign(delta_max, diff)

    # ---------- Configuration API (map these to your config or Modbus) ----------
    # Gains
    def set_outer_gains(self, kp: float, ki: float, kd: float = 0.0, p_on_m: bool = False):
        self.pid_outer.Kp = float(kp)
        self.pid_outer.Ki = float(ki)
        self.pid_outer.Kd = float(kd)
        self.pid_outer.proportional_on_measurement = bool(p_on_m)

    def set_inner_gains(self, kp: float, ki: float, kd: float = 0.0, p_on_m: bool = False):
        self.pid_inner.Kp = float(kp)
        self.pid_inner.Ki = float(ki)
        self.pid_inner.Kd = float(kd)
        self.pid_inner.proportional_on_measurement = bool(p_on_m)

    # Limits & timing
    def set_inner_sp_limits(self, lo: float, hi: float):
        self.inner_sp_min, self.inner_sp_max = float(lo), float(hi)
        self.pid_outer.output_limits = (self.outer_output_min, self.outer_output_max)

    def set_inner_output_limits(self, lo: float, hi: float):
        self.pid_inner.output_limits = (float(lo), float(hi))

    def set_inner_i_limits(self, lo: float, hi: float):
        self.inner_i_min, self.inner_i_max = (float(lo), float(hi))
    
    def set_inner_i_clamp(self, status: bool):
        self.inner_i_clamp = status

    def set_period(self, seconds: float):
        """
        Change controller cadence. Keep aligned with sensor rate if applicable.
        """
        seconds = max(0.1, float(seconds))
        self.pid_period_s = seconds
        self.pid_outer.sample_time = seconds
        self.pid_inner.sample_time = seconds
        self._next_tick = time.monotonic() + seconds

    def set_filters(self, alpha_outer: float, alpha_inner: float):
        self.alpha_outer = float(alpha_outer)
        self.alpha_inner = float(alpha_inner)

    # Modes & setpoints
    def set_outer_mode(self, auto: bool):
        if auto and not self.outer_auto:
            # MANUAL -> AUTO: resume cascade, initialize bumplessly
            self.pid_outer.set_auto_mode(True, last_output=self.inner_sp)
        elif not auto and self.outer_auto:
            # AUTO -> MANUAL: capture current inner SP as the starting manual SP
            # so you can tweak it via set_inner_setpoint() during pre-heat
            self._manual_outer_output_as_inner_sp = self.inner_sp
            self._manual_inner_setpoint = self.inner_sp
            self.pid_outer.set_auto_mode(False)
        self.outer_auto = bool(auto)
  
    def set_inner_mode(self, auto: bool):
        if auto and not self.inner_auto:
            self.pid_inner.set_auto_mode(True, last_output=self.actuator_output)
        elif not auto and self.inner_auto:
           # self._manual_inner_output = self.actuator_output
            self._manual_inner_output = 0
            self.pid_inner.set_auto_mode(False)
        self.inner_auto = bool(auto)

    def set_outer_setpoint(self, target: float):
        self.outer_sp_cmd = float(target)


    def set_inner_setpoint(self, target: float):
        """
        Directly command the inner loop setpoint.
        Effective when the outer loop is MANUAL (not in AUTO cascade mode).
        """
        self._manual_inner_setpoint = float(target)
        # If outer is currently MANUAL, reflect it immediately in the live inner SP
        if not self.outer_auto:
            self.inner_sp = self._clamp(self._manual_inner_setpoint,
                                        self.inner_sp_min, self.inner_sp_max)
            self.pid_inner.setpoint = self.inner_sp


    def set_sp_ramp_rate(self, rate_per_min: float):
        self.outer_sp_ramp_rate_per_min = max(0.0, float(rate_per_min))

    # ---------- Main compute (IO-free) ----------
    def compute(self, outer_pv: float, inner_pv: float, now: Optional[float] = None) -> Tuple[float, float]:
        """
        Perform at most one PID update per 'pid_period_s'.
        Returns (actuator_output, inner_sp).

        You can call this every scan; if the period hasn't elapsed, it simply
        returns the last computed (actuator_output, inner_sp).
        """
        t = time.monotonic() if now is None else now
        if t < self._next_tick:
            return self.actuator_output, self.inner_sp

        # Align to the cadence; catch up if we got behind
        behind = int((t - self._next_tick) // self.pid_period_s) + 1
        self._next_tick += behind * self.pid_period_s

        # Filter PVs
        #self._outer_pv_filt = self._low_pass(self._outer_pv_filt, float(outer_pv), self.alpha_outer)
        #self._inner_pv_filt = self._low_pass(self._inner_pv_filt, float(inner_pv), self.alpha_inner)
        # --- PV VALIDATION AND FILTERING (no early return) ---

        # Validate outer PV
        try:
            opv = float(outer_pv)
            if not math.isfinite(opv) or opv == 0.0 or opv < -10.0 or opv > 120.0:
                opv_valid = False
            else:
                opv_valid = True
        except:
            opv_valid = False

        # Validate inner PV
        try:
            ipv = float(inner_pv)
            if not math.isfinite(ipv) or ipv == 0.0 or ipv < -10.0 or ipv > 120.0:
                ipv_valid = False
            else:
                ipv_valid = True
        except:
            ipv_valid = False

        # Store validity flags
        if opv_valid:
            if not self._outer_pv_valid:
                # First valid reading -> snap
                self._outer_pv_filt = opv
            else:
                # Normal smoothing
                self._outer_pv_filt = self._low_pass(self._outer_pv_filt, opv, self.alpha_outer)

        self._outer_pv_valid = opv_valid

        if ipv_valid:
            if not self._inner_pv_valid:
                self._inner_pv_filt = ipv
            else:
                self._inner_pv_filt = self._low_pass(self._inner_pv_filt, ipv, self.alpha_inner)

        self._inner_pv_valid = ipv_valid

        # --- IMPORTANT ---
        # If PVs are not yet valid, skip PID compute but DON'T early return.
        # Return previous output + previous inner SP.
        if not (self._outer_pv_valid and self._inner_pv_valid):
            return self.actuator_output, self.inner_sp        

        # Outer loop -> compute inner setpoint
        self._update_ramped_outer_sp()
        self.pid_outer.setpoint = self.outer_sp_ramped

        if self.outer_auto:
            inner_sp_cmd = self.pid_outer(self._outer_pv_filt)
        else:
            inner_sp_cmd = self._manual_inner_setpoint
        self.inner_sp = self._clamp(inner_sp_cmd, self.inner_sp_min, self.inner_sp_max)
        self.pid_inner.setpoint = self.inner_sp

        # Inner loop -> compute actuator output
        if self.inner_auto:
            u = self.pid_inner(self._inner_pv_filt)
        else:
            u = self._manual_inner_output



        # Clamp to configured output limits
        lo, hi = self.pid_inner.output_limits
        self.actuator_output = self._clamp(float(u), float(lo), float(hi))

        # Clamp Iacc if I clamping enabled
        if self.inner_i_clamp:
            if abs(self.inner_sp - self._inner_pv_filt) < 1.0:
                self.pid_inner._integral = max(min(self.pid_inner._integral, self.inner_i_max), self.inner_i_min)


        return self.actuator_output, self.inner_sp

    # ---------- Status for HMI/telemetry ----------
    def status(self) -> dict:
        return {
            "outer": {
                "mode": "AUTO" if self.outer_auto else "MANUAL",
                "pv": self._outer_pv_filt,
                "sp_cmd": self.outer_sp_cmd,
                "sp_ramped": self.outer_sp_ramped,
                "inner_sp": self.inner_sp,
                "K": (self.pid_outer.Kp, self.pid_outer.Ki, self.pid_outer.Kd),
                "Iacc":(self.pid_outer._integral),

            },
            "inner": {
                "mode": "AUTO" if self.inner_auto else "MANUAL",
                "pv": self._inner_pv_filt,
                "sp": self.pid_inner.setpoint,
                "output": self.actuator_output,
                "K": (self.pid_inner.Kp, self.pid_inner.Ki, self.pid_inner.Kd),
                "Iacc":(self.pid_inner._integral),
            },
            "period_s": self.pid_period_s,
        }
    