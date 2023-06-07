# https://pygraphviz.github.io/documentation/stable/tutorial.html
import pygraphviz as pgv
import sys

class MotionExecuteGraphGenerator(object):
  def __init__(self, fname=None, state_outcomes=None):
    self.graph = pgv.AGraph(fname)

    self.dest_list = {}
    self.edge_list = {}
    if state_outcomes is None:
      self.outcome_states = [ self.graph.nodes()[-1] ]
    else:
      self.outcome_states = []
      for nm in state_outcomes:
        for n in self.graph.nodes():
          if n.get_name() == nm:
            self.outcome_states.append(n)
            break

    for e in self.graph.edges_iter():
      #e.label = e.attr['label']
      label = e.attr['label']
      if label is None or len(label) == 0:
        label = '{}_{}'.format(e[0], e[1])
      setattr(e, 'label', label)
      src = e[0]
      dst = e[1]
      if src in self.dest_list:
        lst = self.dest_list[src]
        lst.append(dst)
        self.dest_list[src] = lst
        elst = self.edge_list[src]
        elst.append(e)
        self.edge_list[src] = elst
      else:
        self.dest_list[src] = [dst]
        self.edge_list[src] = [e]

  def extract_outcomes(self, shape='box'):
    self.outcome_states = []
    for node in self.graph.nodes_iter():
      if node.attr['shape'] == shape:
        self.outcome_states.append(node)

  def print_smach_definition(self, strm=sys.stdout):
    self.printHeader(strm=strm)
    for n in self.graph.nodes_iter():
      if n in self.outcome_states:
        continue
      self.printNodes(n, strm=strm)
    self.printMachine(strm=strm)

  def print_motion_library_template(self, strm=sys.stdout):
    print('import random', file=strm)
    print('import rospy', file=strm)
    print('class MotionLib(object):', file=strm)
    print('  def __init__(self):', file=strm)
    print('    pass', file=strm)
    for node in self.graph.nodes_iter():
      if node in self.outcome_states:
        continue
      print('  def {}_func(self, **agrs):'.format(node.get_name()), file=strm)
      print('    rospy.sleep(1)', file=strm)
      size = len(self.edge_list[node])
      print('    pos = int({} * random.random())'.format(size), file=strm)
      cntr = 0
      for edge in self.edge_list[node]:
        print('    if pos == {}:'.format(cntr), file=strm)
        print('      return "{}"'.format(edge.label), file=strm)
        cntr += 1
      print('    return "fail"', file=strm)

  def printHeader(self, strm=sys.stdout):
    print('import smach', file=strm)

  def printNodes(self, node, strm=sys.stdout):
    outstr = ''
    for edge in self.edge_list[node]:
      outstr = outstr + '"{}", '.format(edge.label)

    print('class {}_state(smach.State):'.format(node.get_name()), file=strm)
    print('    def __init__(self,lib=None):', file=strm)
    print('        smach.State.__init__(self, outcomes=[{}],'.format(outstr), file=strm)
    print('                             input_keys=["udata"], output_keys = ["udata"])', file=strm)
    print('        self.name = "{}"'.format(node.get_name()), file=strm)
    print('        self.motionlib = lib', file=strm)
    print('    def execute(self, userdata):', file=strm)
    print('        userdata.udata = userdata.udata + "->" + self.name', file=strm)
    print('        if self.motionlib:', file=strm)
    print('            return self.motionlib.{}_func(userdata=userdata)'.format(node.get_name()), file=strm)
    print('        return "fail"', file=strm)

  def printMachine(self, strm=sys.stdout):
    print('def create_state_machine(lib = None):', file=strm)
    #print('    smach.set_loggers(smach.loginfo, smach.logwarn, log_none, smach.logerr)', file=strm)
    print('    sm = smach.StateMachine(outcomes=[', file=strm)
    for node in self.outcome_states:
      print('                                   "{}",'.format(node.get_name()), file=strm)
    print('                                     ])', file=strm)
    print('    with sm:', file=strm)
    for node in self.graph.nodes_iter():
      if node in self.outcome_states:
        continue
      print('        smach.StateMachine.add("{}", {}_state(lib),'.format(node.get_name(), node.get_name()), file=strm)
      print('                               transitions={', file=strm)
      for edge in self.edge_list[node]:
        print('                                            "{}":"{}",'.format(edge.label, edge[1].get_name()), file=strm)
      print('                                            } )', file=strm)
    print('    sm.userdata.udata = ""', file=strm)
    print('    return sm', file=strm)
