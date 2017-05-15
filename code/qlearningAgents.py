#############################
my_univ_id = "2011118035"     ## Univ. Num.
my_password = "01044752273"   ## Your Phone Number
#############################

# qlearningAgents.py
# ------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from game import *
from learningAgents import ReinforcementAgent
from featureExtractors import *

import random,util,math

debug = True

class QLearningAgent(ReinforcementAgent):
    """
      Q-Learning Agent

      Functions you should fill in:
        - computeValueFromQValues
        - computeActionFromQValues
        - getQValue
        - getAction
        - update

      Instance variables you have access to
        - self.epsilon (exploration prob)
        - self.alpha (learning rate)
        - self.discount (discount rate)

      Functions you should use
        - self.getLegalActions(state)
          which returns legal actions for a state
    """
    def __init__(self, **args):
        "You can initialize Q-values here,,,"
        ReinforcementAgent.__init__(self, **args)
        if debug == True:
            print("ALPHA", self.alpha)
            print("DISCOUNT", self.discount)
            print("EXPLORATION", self.epsilon)
        self.qValues = util.Counter()

    def getQValue(self, state, action):
        """
          Returns Q(state,action)
          Should return 0.0 if we have never seen a state
          or the Q node value otherwise
        """

        "*** YOUR CODE HERE ***"
        key = (state,action)
        if key not in self.qValues.keys():
            return 0.0
        else:
            return self.qValues[key]
       
    def computeValueFromQValues(self, state):
        """
          Returns max_action Q(state,action)
          where the max is over legal actions.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return a value of 0.0.
        """

        "*** YOUR CODE HERE ***"
        qVals = []
        legalActions = self.getLegalActions(state)
        for legalAction in legalActions:
            qVals.append(self.getQvalue(state,legalAction))
        if len(legalActions) == 0:
            return 0.0
        else :
            return max(qVals)
            

    def computeActionFromQValues(self, state):
        """
          Compute the best action to take in a state.  Note that if there
          are no legal actions, which is the case at the terminal state,
          you should return None.
        """

        "*** YOUR CODE HERE ***"
        actions = self.getLegalActions(state)
        if len(actions) == 0:
            return None
        max_action = None
        max_qval = float('inf') * (-1.0)
        for action in actions:
            qval = self.getQValue(state,action)
            if qval > max_qval :
                max_qval = qval
                max_action = action
           
        return max_action
            

    def getAction(self, state):
        """
          Compute the action to take in the current state.  With
          probability self.epsilon, we should take a random action and
          take the best policy action otherwise.  Note that if there are
          no legal actions, which is the case at the terminal state, you
          should choose None as the action.

          HINT: You might want to use util.flipCoin(prob)
          HINT: To pick randomly from a list, use random.choice(list)
        """
        # Pick Action
        legalActions = self.getLegalActions(state)
        action = None
            
        "*** YOUR CODE HERE ***"
        if util.flipCoin(self.epsilon):
            action = random.choice(legalActions)
        else:
            action = self.computeActionFromQValues(state)
        return action
        

    def update(self, state, action, nextState, reward):
        """
          The parent class calls this to observe a
          state = action => nextState and reward transition.
          You should do your Q-Value update here

          NOTE: You should never call this function,
          it will be called on your behalf
        """
        if debug == True:
            print("State: ", state, " , Action: ", action, " , NextState: ", nextState, " ,) Reward: ", reward)
            print("QVALUE", self.getQValue(state, action))
            print("VALUE", self.getValue(nextState))

        "*** YOUR CODE HERE ***"
        qval = self.qValues[(state,action)]
        #get max_action if nextState is terminal return None
        nextAction = self.computeActionFromQValues(nextState) 
        if nextAction is None :
            sample = reward
        else:
            sample = reward + self.discount * self.getQValue(nextState, nextAction)
        qval += self.alpha *(sample - qval) 
        
        self.qValues[(state,action)] = qval


    def getPolicy(self, state):
        return self.computeActionFromQValues(state)

    def getValue(self, state):
        return self.computeValueFromQValues(state)


class PacmanQAgent(QLearningAgent):
    "Exactly the same as QLearningAgent, but with different default parameters"

    def __init__(self, epsilon=0.05,gamma=0.8,alpha=0.2, numTraining=0, **args):
        """
        These default parameters can be changed from the pacman.py command line.
        For example, to change the exploration rate, try:
            python pacman.py -p PacmanQLearningAgent -a epsilon=0.1

        alpha    - learning rate
        epsilon  - exploration rate
        gamma    - discount factor
        numTraining - number of training episodes, i.e. no learning after these many episodes
        """
        args['epsilon'] = epsilon
        args['gamma'] = gamma
        args['alpha'] = alpha
        args['numTraining'] = numTraining
        self.index = 0  # This is always Pacman
        QLearningAgent.__init__(self, **args)

    def getAction(self, state):
        """
        Simply calls the getAction method of QLearningAgent and then
        informs parent of action for Pacman.  Do not change or remove this
        method.
        """
        action = QLearningAgent.getAction(self,state)
        self.doAction(state,action)
        return action


class ApproximateQAgent(PacmanQAgent):
    """
       ApproximateQLearningAgent

       You should only have to overwrite getQValue
       and update.  All other QLearningAgent functions
       should work as is.
    """
    def __init__(self, extractor='IdentityExtractor', **args):
        self.featExtractor = util.lookup(extractor, globals())()
        PacmanQAgent.__init__(self, **args)
        self.weights = util.Counter()

    def getWeights(self):
        return self.weights

    def getQValue(self, state, action):
        """
          Should return Q(state,action) = w * featureVector
          where * is the dotProduct operator
        """
        #get features
        features = self.featExtractor.getFeatures(state,action)
        #get weights
        weights = self.getWeights()
        #comupute dot product or features and weights
        qval = 0
        for feature in features.keys():
            qval += weights[feature] * features[feature]
        
        return qval
            


    def update(self, state, action, nextState, reward):
        """
           Should update your weights based on transition
        """
        "*** YOUR CODE HERE ***"
        features = self.featExtractor.getFeatures(state,action)
        legalActions = self.getLegalActions(nextState)
        #derivative respect to weight m :
        #[r + gamma * max_a'_Q(s',a')-] *fm(s,a)
        difference = 0
        qsa = self.getQValue(state,action)
        weights = self.getWeights()
        if len(legalActions) == 0:
            difference = reward - qsa
        else:
            target = max([self.getQValue(nextState,nextAction) \
                          for nextAction in legalActions])
            difference = (reward + self.discount *target )-qsa    
        
        for feature in features.keys(): 
            weights[feature] += self.alpha * difference * features[feature]
        
    
    def final(self, state):
        "Called at the end of each game."

        # Set id and password
        if state.univ_id == '' and state.password == '':
            state.univ_id = my_univ_id
            state.password = my_password

        # call the super-class final method
        PacmanQAgent.final(self, state)

        # did we finish training?
        if self.episodesSoFar == self.numTraining:
            # you might want to print your weights here for debugging
            "*** YOUR CODE HERE (Use only when you need it. Not required)***"
            pass
