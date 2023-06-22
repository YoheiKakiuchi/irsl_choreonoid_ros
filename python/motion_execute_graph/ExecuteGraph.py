# https://pygraphviz.github.io/documentation/stable/tutorial.html
import pygraphviz as pgv

class StateMachine(object):
    def __init__(self, name, outcomes=[]):
        self.name = name
        if len(outcomes) == 0:
            raise Exception('StateMachine should have outcomes : {}'.format(outcomes))
        self.outcomes = outcomes
        self.states = {}
        self.connections = {}
        self.depth = 0

    def hasChild(self):
        return len(self.states) != 0

    def isTop(self):
        return self.depth == 0

    def addState(self, state):
        if state.name in self.states:
            raise Exception('State : {} already exists. Name of a state should be unique.'.format(state.name))
        self.states[state.name] = state
        state.depth = self.depth + 1

    def addConnection(self, source_name, outcome, destination_name, name=None):
        if not source_name in self.states:
            raise Exception('State : {} does not exist'.format(source_name))
        source = self.states[source_name]
        if not outcome in source.outcomes:
            raise Exception('Outcome : {} does not exist in State : {}'.format(outcome, source.name))
        sc = None
        if destination_name in self.states:
            sc = StateConnection(source, outcome, self.states[destination_name])
        elif destination_name in outcomes:
            sc = StateConnection(source, outcome, destination_name)
        else:
            raise Exception('State : {} does not exist.'.format(destination_name))

    def storeAsDot(self, fname, depth=0):
        pass

    def generateSmachFiles(self):
        pass

    @classmethod
    def createFromDot(cls, name=None):
        '''read as base-action (not layered state-machine)'''
        pass

class StateConnection(object):
    def __init__(self, source, outcome, destination, name=None):
        if name is None:
            if type(destination) is str:
                self.name = '{}_{}_{}'.format(source.name, outcome, destination)
            else:
                self.name = '{}_{}_{}'.format(source.name, outcome, destination.name)
        else:
            self.name = name
        self.source = source
        self.outcome = outcome
        self.destination = destination
###
### example
###
# a = StateMachine.creteFromDot('a.dot', 'a_action')
# b = StateMachine.creteFromDot('b.dot', 'b_action')
# sm = StateMachine('top', ['success', 'fail'])
# sm.addState(a)
# sm.addState(b)
# sm.addConnection(a, 'success', b)
# sm.addConnection(a, 'fail', 'fail')
# sm.addConnection(b, 'success', 'success')
# sm.addConnection(b, 'fail', 'fail')
# sm.generateSmachFiles()

## base-action
