import os
import pygame
import time
import logging
from dataclasses import dataclass
from typing import Optional, Dict, List, Any
from dadou_utils_ros.utils_static import USB_GAMEPAD

@dataclass
class ControllerMapping:
    # Required
    a_button: int
    lx_axis: int
    ly_axis: int
    # Optional buttons
    b_button: Optional[int] = None
    x_button: Optional[int] = None
    y_button: Optional[int] = None
    start_button: Optional[int] = None
    select_button: Optional[int] = None
    l1_button: Optional[int] = None
    r1_button: Optional[int] = None
    l2_button: Optional[int] = None
    r2_button: Optional[int] = None
    # Optional axes
    rx_axis: Optional[int] = None
    ry_axis: Optional[int] = None
    l2_axis: Optional[int] = None
    r2_axis: Optional[int] = None
    # Axis orientation
    invert_ly: bool = True
    invert_ry: bool = True


MAPPINGS: Dict[str, ControllerMapping] = {
    # Xbox-like (souvent A=0, B=1, X=2, Y=3)
    "xbox": ControllerMapping(
        a_button=0,
        b_button=1,
        x_button=2,
        y_button=4,
        start_button=7,
        select_button=6,
        l1_button=40,
        r1_button=5,
        # L2/R2 analog triggers commonly on axes 2 and 5
        l2_axis=2,
        r2_axis=5,
        lx_axis=0,
        ly_axis=1,
        rx_axis=3,
        ry_axis=4,
    ),
    # PlayStation (souvent Cross/X=0, Circle=1, Square=2, Triangle=3)
    # Ici, on mappe:
    #   A -> Cross (0), B -> Circle (1), X -> Square (2), Y -> Triangle (3)
    #   START -> Options (9), SELECT -> Share (8)
    "playstation": ControllerMapping(
        a_button=0,
        b_button=1,
        x_button=2,
        y_button=3,
        start_button=9,
        select_button=8,
        l1_button=4,
        r1_button=5,
        # DS4/DS5 often expose L2/R2 as axes; choose indices avoiding rx/ry
        l2_axis=3,
        r2_axis=4,
        lx_axis=0,
        ly_axis=1,
        rx_axis=2,
        ry_axis=5,
    ),
    # Nintendo/Switch-Pro (souvent B=0, A=1, Y=2, X=3)
    # On mappe la touche "A" logique sur index 1.
    # START -> Plus (+) = 9, SELECT -> Moins (-) = 8
    "switch": ControllerMapping(
        a_button=1,
        b_button=0,
        x_button=3,
        y_button=2,
        start_button=9,
        select_button=8,
        l1_button=4,
        r1_button=5,
        l2_axis=4,
        r2_axis=5,
        lx_axis=0,
        ly_axis=1,
        rx_axis=2,
        ry_axis=3,
    ),
    # GameCube via adaptateur (varie selon drivers; hypothèse courante)
    "gamecube": ControllerMapping(
        a_button=0,
        b_button=1,
        x_button=3,
        y_button=2,
        start_button=7,
        select_button=None,
        l1_button=None,
        r1_button=None,
        l2_axis=4,
        r2_axis=5,
        lx_axis=0,
        ly_axis=1,
        rx_axis=2,
        ry_axis=3,
    ),
    # Fallback générique
    "generic": ControllerMapping(
        a_button=0,
        b_button=1,
        x_button=2,
        y_button=3,
        start_button=7,
        select_button=6,
        l1_button=4,
        r1_button=5,
        l2_axis=2,
        r2_axis=5,
        lx_axis=0,
        ly_axis=1,
        rx_axis=2,
        ry_axis=3,
    ),
}


BUTTON_KEYS = (
    "A",
    "B",
    "X",
    "Y",
    "L1",
    "L2",
    "R1",
    "R2",
    "START",
    "SELECT",
    "UP",
    "DOWN",
    "LEFT",
    "RIGHT",
)

JOYSTICK_KEYS = ("LX", "LY", "RX", "RY")


class USBGamepad:
    logger = logging.getLogger()

    def __init__(self, controller_type: str = "auto"):
        os.environ["SDL_AUDIODRIVER"] = "dummy"
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        pygame.init()
        #if pygame.mixer.get_init():
        #    pygame.mixer.quit()
        #try:
        #    pygame.display.set_mode((1, 1))
        #except Exception:
        #    pass
        #pygame.joystick.init()

        self.gamepad: Optional[pygame.joystick.Joystick] = None
        self._warned_no_gamepad = False
        self.controller_type = controller_type.lower() if controller_type else "auto"
        self.mapping: ControllerMapping = MAPPINGS["generic"]
        # Neutral references to filter hardware bias/drift
        self._axis_neutral: Dict[str, Optional[int]] = {axis: None for axis in JOYSTICK_KEYS}
        self._axis_neutral_snap = 4
        self._trigger_neutral: Dict[str, Optional[float]] = {"L2": None, "R2": None}
        self._trigger_threshold = 0.25

        if pygame.joystick.get_count() == 0:
            logging.info("Aucune manette détectée.")
            return

        self.gamepad = pygame.joystick.Joystick(0)
        self.gamepad.init()
        name = self.gamepad.get_name() or ""
        logging.info(f"Manette détectée : {name}")

        detected_type = self._detect_type_from_name(name) if self.controller_type == "auto" else self.controller_type
        self.mapping = MAPPINGS.get(detected_type, MAPPINGS["generic"])
        logging.info(f"Profil manette: {detected_type}")

    def _detect_type_from_name(self, name: str) -> str:
        n = name.lower()
        if any(k in n for k in ["xbox", "x-input", "xinput", "x-box", "shanwan"]):
            return "xbox"
        if any(k in n for k in ["playstation", "dualshock", "dual sense", "dualsense", "sony"]):
            return "playstation"
        if any(k in n for k in ["switch", "nintendo", "pro controller"]):
            return "switch"
        if "gamecube" in n:
            return "gamecube"
        return "generic"

    def _scale_axis_to_0_99(self, v: float) -> int:
        # pygame retourne [-1.0, 1.0] -> on mappe sur [0, 99]
        s = int(round(((v + 1.0) / 2.0) * 99))
        return max(0, min(99, s))

    def _update_axis_neutral(self, axis: str, value: Optional[int]) -> None:
        if value is None:
            return
        neutral = self._axis_neutral.get(axis)
        if neutral is None:
            self._axis_neutral[axis] = value
            return
        if abs(value - neutral) <= self._axis_neutral_snap:
            self._axis_neutral[axis] = int(round(neutral * 0.8 + value * 0.2))

    def _neutral_value(self, axis: str) -> int:
        neutral = self._axis_neutral.get(axis)
        if neutral is None:
            return 50
        return neutral

    def _update_neutral_from_state(self, state: Dict[str, int]) -> None:
        for axis in JOYSTICK_KEYS:
            value = state.get(axis)
            if isinstance(value, int):
                self._update_axis_neutral(axis, value)

    def _trigger_from_axis(self, idx: Optional[int], name: str) -> int:
        if idx is None:
            return 0
        try:
            v = self.gamepad.get_axis(idx)
        except Exception:
            return 0
        norm = (v + 1.0) / 2.0
        norm = max(0.0, min(1.0, norm))
        neutral = self._trigger_neutral.get(name)
        if neutral is None:
            self._trigger_neutral[name] = norm
            return 0
        delta = norm - neutral
        if abs(delta) >= self._trigger_threshold:
            return 1
        self._trigger_neutral[name] = neutral * 0.9 + norm * 0.1
        return 0

    def get_normalized_state(self) -> Optional[Dict[str, int]]:
        if not self.gamepad:
            return {
                "connected": 0,
                "A": 0,
                "B": 0,
                "X": 0,
                "Y": 0,
                "L1": 0,
                "L2": 0,
                "R1": 0,
                "R2": 0,
                "START": 0,
                "SELECT": 0,
                "UP": 0,
                "DOWN": 0,
                "LEFT": 0,
                "RIGHT": 0,
                "LX": 0,
                "LY": 0,
                "RX": 0,
                "RY": 0,
            }
        try:
            pygame.event.pump()
        except Exception:
            return None

        state = (
            "Pad %s | buttons=%s | axes=%s | hats=%s",
            self.gamepad.get_name(),
            list(map(self.gamepad.get_button, range(self.gamepad.get_numbuttons()))),
            [f"{self.gamepad.get_axis(i):+.2f}" for i in range(self.gamepad.get_numaxes())],
            list(map(self.gamepad.get_hat, range(self.gamepad.get_numhats())))
        )

        m = self.mapping

        def get_button(idx: Optional[int]) -> int:
            if idx is None:
                return 0
            try:
                return 1 if self.gamepad.get_button(idx) else 0
            except Exception:
                return 0

        btnA = get_button(m.a_button)
        btnB = get_button(m.b_button)
        btnX = get_button(m.x_button)
        btnY = get_button(m.y_button)
        btnSTART = get_button(m.start_button)
        btnSELECT = get_button(m.select_button)
        btnL1 = get_button(m.l1_button)
        btnR1 = get_button(m.r1_button)
        # L2/R2 can be on buttons or axes; pressed if either is active
        btnL2_b = get_button(m.l2_button)
        btnR2_b = get_button(m.r2_button)

        def get_axis(idx: Optional[int]) -> int:
            if idx is None:
                return 0
            try:
                return self._scale_axis_to_0_99(self.gamepad.get_axis(idx))
            except Exception:
                return 0

        lx = get_axis(m.lx_axis)
        ly = get_axis(m.ly_axis)
        rx = get_axis(m.rx_axis)
        ry = get_axis(m.ry_axis)

        if m.invert_ly:
            ly = 99 - ly
        if m.invert_ry:
            ry = 99 - ry

        for axis_name, axis_value in ("LX", lx), ("LY", ly), ("RX", rx), ("RY", ry):
            self._update_axis_neutral(axis_name, axis_value)

        # D-Pad via HAT si disponible
        up = down = left = right = 0
        try:
            if self.gamepad.get_numhats() > 0:
                hx, hy = self.gamepad.get_hat(0)
                left = 1 if hx < 0 else 0
                right = 1 if hx > 0 else 0
                up = 1 if hy > 0 else 0
                down = 1 if hy < 0 else 0
        except Exception:
            pass

        btnL2 = btnL2_b or self._trigger_from_axis(m.l2_axis, "L2")
        btnR2 = btnR2_b or self._trigger_from_axis(m.r2_axis, "R2")

        return {
            "connected": 1,
            "A": btnA,
            "B": btnB,
            "X": btnX,
            "Y": btnY,
            "L1": btnL1,
            "L2": btnL2,
            "R1": btnR1,
            "R2": btnR2,
            "START": btnSTART,
            "SELECT": btnSELECT,
            "UP": up,
            "DOWN": down,
            "LEFT": left,
            "RIGHT": right,
            "LX": lx,
            "LY": ly,
            "RX": rx,
            "RY": ry,
        }

    def active_inputs(
        self,
        state: Optional[Dict[str, int]] = None,
        deadzone: int = 10,
    ) -> Optional[Dict[str, Any]]:
        if state is None:
            state = self.get_normalized_state()

        if state is None:
            return None

        self._update_neutral_from_state(state)

        buttons = [btn for btn in BUTTON_KEYS if state.get(btn) == 1]

        active_axes: Dict[str, int] = {}
        for axis in JOYSTICK_KEYS:
            value = state.get(axis)
            if not isinstance(value, int):
                continue
            neutral = self._neutral_value(axis)
            if abs(value - neutral) >= deadzone:
                active_axes[axis] = value

        return {"buttons": buttons, "joysticks": active_axes}

    def check_inputs(self):
        if not self.gamepad:
            if getattr(self, "_warned_no_gamepad", False) is False:
                logging.info("Aucune manette détectée: check() ignoré.")
                self._warned_no_gamepad = True
            return

        state = self.get_normalized_state()
        if state is None:
            return None

        active_inputs = self.active_inputs(state) or {"buttons": [], "joysticks": {}}

        if active_inputs["buttons"] or active_inputs["joysticks"]:
            buttons_str = ", ".join(active_inputs["buttons"]) or "aucun"
            joystick_details = (
                ", ".join(f"{axis}={value}" for axis, value in active_inputs["joysticks"].items())
                or "aucun"
            )
            logging.info(
                "Entrées actives : boutons [%s], joysticks [%s]",
                buttons_str,
                joystick_details,
            )
            return {USB_GAMEPAD : active_inputs}
