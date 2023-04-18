from Behavior import *

class Template_Controller(Controller):
    def Loop(self, x) -> None:
        Contemp = self.iface.get_random_state_space()
        print(f"{Contemp[0]}, {Contemp[1]}, {Contemp[2]}")
        if Contemp[2] > 1:
            self.Complete=True
        return
    def SafeExit(self, x) -> None:
        print("No pls")
        return
    
class Forward(Controller):
    def Loop(self, x) -> None:
        Contemp = self.iface.get_random_state_space()
        print(f"{Contemp[0]}, {Contemp[1]}, {Contemp[2]}")
        if Contemp[2] > 1:
            self.Complete=True
        return
    def SafeExit(self, x) -> None:
        print("No pls")
        return