from abc import ABC, abstractmethod
import dbus

class Controller(ABC):
    iface    : dbus.Interface
    ifaceAI  : dbus.Interface
    odrv0    : object
    Error    : str = None
    Complete : bool = False

    wheel_distance : float = 0.235
    wheel_radius   : float = 0.084

    def __init__(self, iface, ifaceAI, odrv0) -> None:
        self.iface = iface
        self.ifaceAI = ifaceAI
        self.odrv0 = odrv0

    @abstractmethod
    def Loop(self) -> None:
        pass
    @abstractmethod
    def SafeExit(self) -> None:
        pass

    def ControlLoop(self, CancelRequired : bool) -> None:
        if CancelRequired:
            self.SafeExit()
        else:
            self.Loop()

    def rotate(self, omega):
        self.odrv0.axis0.controller.input_vel = omega*self.wheel_distance/self.wheel_radius
        self.odrv0.axis1.controller.input_vel = omega*self.wheel_distance/self.wheel_radius

    def move(self, velocity):
        self.odrv0.axis0.controller.input_vel = -2*velocity/self.wheel_radius
        self.odrv0.axis1.controller.input_vel = 2*velocity/self.wheel_radius
    
    def stop(self):
        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0


class TurnAbsolute(Controller):
    goal : float
    K_p  : float

    ERROR_MARGINE : float = 0.1

    def __init__(self, iface, ifaceAI, odrv0, goal) -> None:
        super().__init__(iface, ifaceAI, odrv0)
        self.goal = goal

    def Loop(self) -> None:
        x, y, theta = self.iface.get_random_state_space()

        # Check if the goal is reached
        e = self.goal - theta
        if e < self.ERROR_MARGINE:
            self.stop()
            self.Complete = True
            return

        # Calculate omega using PID
        omega = self.K_p*e

        # Set omega
        omega = 1
        self.rotate(omega)

        return
    
    def SafeExit(self) -> None:
        self.stop()
        print("Stopped due to canceling behaviour.")
        return

class Template_Controller(Controller):
    def Loop(self) -> None:
        Contemp = self.iface.get_random_state_space()
        print(f"{Contemp[0]}, {Contemp[1]}, {Contemp[2]}")
        if Contemp[2] > 1:
            self.Complete=True
        return
    def SafeExit(self) -> None:
        print("No pls")
        return