import smach
class stop_state(smach.State):
    def __init__(self,lib=None):
        smach.State.__init__(self, outcomes=["stop_forward", "stop_back", "stop_end", ],
                             input_keys=["udata"], output_keys = ["udata"])
        self.name = "stop"
        self.motionlib = lib
    def execute(self, userdata):
        userdata.udata = userdata.udata + "->" + self.name
        if self.motionlib:
            return self.motionlib.stop_func(userdata=userdata)
        return "fail"
class forward_state(smach.State):
    def __init__(self,lib=None):
        smach.State.__init__(self, outcomes=["forward_stop", "forward_back", ],
                             input_keys=["udata"], output_keys = ["udata"])
        self.name = "forward"
        self.motionlib = lib
    def execute(self, userdata):
        userdata.udata = userdata.udata + "->" + self.name
        if self.motionlib:
            return self.motionlib.forward_func(userdata=userdata)
        return "fail"
class back_state(smach.State):
    def __init__(self,lib=None):
        smach.State.__init__(self, outcomes=["back_stop", "back_forward", ],
                             input_keys=["udata"], output_keys = ["udata"])
        self.name = "back"
        self.motionlib = lib
    def execute(self, userdata):
        userdata.udata = userdata.udata + "->" + self.name
        if self.motionlib:
            return self.motionlib.back_func(userdata=userdata)
        return "fail"
def create_state_machine(lib = None):
    sm = smach.StateMachine(outcomes=[
                                   "end",
                                     ])
    with sm:
        smach.StateMachine.add("stop", stop_state(lib),
                               transitions={
                                            "stop_forward":"forward",
                                            "stop_back":"back",
                                            "stop_end":"end",
                                            } )
        smach.StateMachine.add("forward", forward_state(lib),
                               transitions={
                                            "forward_stop":"stop",
                                            "forward_back":"back",
                                            } )
        smach.StateMachine.add("back", back_state(lib),
                               transitions={
                                            "back_stop":"stop",
                                            "back_forward":"forward",
                                            } )
    sm.userdata.udata = ""
    return sm
