from abc import ABC, abstractmethod
import dbus

class Controller(ABC):
    iface : dbus.Interface
    ifaceAI : dbus.Interface
    Error : str = None
    Complete : bool = False
    @abstractmethod
    def Loop(self, x) -> None:
        pass
    @abstractmethod
    def SafeExit(self, x) -> None:
        pass
    def __init__(self, iface : dbus.Interface = None, ifaceAI : dbus.Interface = None) -> None:
        self.iface = iface
        self.ifaceAI = ifaceAI
    def ControlLoop(self,x,e : bool) -> None:
        if e:
            self.SafeExit(x)
        else:
            self.Loop(x)
    