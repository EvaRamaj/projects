import rospy
import numpy as np
import threading
import sys
from connect4_detector import *

DO_UNIFORM_UPDATE = True


class Connect4Filter(Connect4Detector):
    def __init__(self, detector):
        self.reset()
        self.detector = detector

        self.do_uniform_update = DO_UNIFORM_UPDATE

        #params for model update
        self.chance_wrong_column = 0.03
        self.chance_piece_dropped = 0.01
        self.chance_piece_correct = 1.0 - self.chance_wrong_column*2 - self.chance_piece_dropped

    def reset(self):
        #probability of piece at (c,r) being
        #empty: board_belief[c,r,0]
        #red: board_belief[c,r,1]
        #blue: board_belief[c,r,2]
        self.board_belief = np.zeros((COLUMNS, ROWS,3), dtype=np.float64)
        #self.board_belief[:,:,0] = 1.0
        #self.board_belief[:,:,1:] = 0.0
        self.board_belief[:,:,0] = 0.9
        self.board_belief[:,:,1:] = 0.05

    def detect_state(self, sim):
        self.sensorUpdate(get_img())
        return self.getMostLikelyBoard()

    def calculate_probs(self, img):
        return self.detector.calculate_probs(img)

    def getMostLikelyBoard(self):
        return np.argmax(self.board_belief, axis=2)

    #sensor update, has to be overwritten by child classes
    def sensorUpdate(self, img):
        raise NotImplemented("Needs to be overwritten by child class!")

    #default model update, can be overwritten by child classes
    def modelUpdate(self, lastMove):
        #update probabilities using the model prediciton
        lastColumn, lastRow, lastPlayer = lastMove
        if lastPlayer<0: #at least player needs to be known
            print("No player specified!")
            return
        if lastColumn<0 or lastRow<0:
            if self.do_uniform_update: self._modelUpdateUniform(lastPlayer)
            return

        self._modelUpdateColumn(lastPlayer, lastColumn, self.chance_piece_correct)
        if lastColumn+1 < COLUMNS:
            self._modelUpdateColumn(lastPlayer, lastColumn+1, self.chance_wrong_column)
        if lastColumn-1 >= 0:
            self._modelUpdateColumn(lastPlayer, lastColumn-1, self.chance_wrong_column)

    def _modelUpdateUniform(self, player):
        #update probs s.t. a piece might hav dropped anywhere
        chance_uniform = 1./COLUMNS
        for c in range(COLUMNS): self._modelUpdateColumn(player, c, chance_uniform)

    def _modelUpdateColumn(self, player, column, chance):
        #module update for said column
        c,p = column,player
        for r in range(ROWS-1, 0, -1):
            self.board_belief[c,r,p] = 1.0*self.board_belief[c,r,p] + chance*self.board_belief[c,r,0]*(1. - self.board_belief[c,r-1,0])
            self.board_belief[c,r,0] = self.board_belief[c,r,0]*self.board_belief[c,r-1,0] + (1-chance)*self.board_belief[c,r,0]*(1. - self.board_belief[c,r-1,0])
        r = 0
        self.board_belief[c,r,p] = 1.0*self.board_belief[c,r,p] + chance*self.board_belief[c,r,0]
        self.board_belief[c,r,0] = (1-chance)*self.board_belief[c,r,0]



#Simple filter for connect 4 (averages over sensor, ignores model)
class SimpleFilter(Connect4Filter):
    def __init__(self, detector):
        super(SimpleFilter, self).__init__(detector)
        self.sensor_weight = 0.3

    def sensorUpdate(self, img):
        probs = self.detector.calculate_probs(img)
        self.board_belief = self.sensor_weight*probs + (1-self.sensor_weight)*self.board_belief

    def modelUpdate(self, lastMove):
        pass

#Average filter for connect 4 (like simple filter, but with model update)
class AverageFilter(Connect4Filter):
    def __init__(self, detector):
        super(AverageFilter, self).__init__(detector)
        self.sensor_weight = 0.3

    def sensorUpdate(self, img):
        probs = self.detector.calculate_probs(img)
        self.board_belief = self.sensor_weight*probs + (1-self.sensor_weight)*self.board_belief


#Bayes filter for connect 4
class BayesFilter(Connect4Filter):
    def __init__(self, detector):
        super(BayesFilter, self).__init__(detector)

        #params for model update
        self.chance_wrong_column = 0.01
        self.chance_piece_dropped = 0.01
        self.chance_piece_correct = 1.0 - self.chance_wrong_column*6 - self.chance_piece_dropped


    def sensorUpdate(self, img):
        #TODO update probabilities using the sensor data
        probs = self._sensorProbs(img)

        #the correct update for a bayes filter:
        self.board_belief *= probs
        self.board_belief /= np.expand_dims(np.sum(self.board_belief, axis=2), axis=2)

    def _sensorProbs(self, img):
        probs = self.detector.calculate_probs(img)
        sensor_data = np.argmax(probs, axis=2)
        #TODO calc p(z|x)
        for c in range(COLUMNS):
            for r in range(ROWS):
                if sensor_data[c,r] == 0:
                    probs[c,r,0] = 0.8
                    probs[c,r,1] = 0.05
                    probs[c,r,2] = 0.05
                if sensor_data[c,r] == 1:
                    probs[c,r,0] = 0.1
                    probs[c,r,1] = 0.9
                    probs[c,r,2] = 0.05
                if sensor_data[c,r] == 2:
                    probs[c,r,0] = 0.1
                    probs[c,r,1] = 0.05
                    probs[c,r,2] = 0.9
        return probs


    #default model update, can be overwritten by child classes
    def modelUpdate(self, lastMove):
        #update probabilities using the model prediciton
        lastColumn, lastRow, lastPlayer = lastMove
        if lastPlayer<0: #at least player needs to be known
            print("No player specified!")
            return

        self._modelUpdateUniform(1)
        self._modelUpdateUniform(2)
        self._modelUpdateUniform(lastPlayer)

        #if lastColumn<0 or lastRow<0:
        #    #we'll do it twice for bayes filter
        #    self._modelUpdateUniform(lastPlayer)
        #    return

        #for c in range(COLUMNS):
        #    chance = self.chance_piece_correct if c == lastColumn else self.chance_wrong_column
        #    self._modelUpdateColumn(lastPlayer, c, chance)

        


#Threaded filter, starts a thread that continously updates sensor model
class ThreadedFilter(Connect4Filter):
    def __init__(self, filterDetector):
        self.filter = filterDetector
        self.rate = rospy.Rate(1.0)
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update_thread)
        self.thread.daemon = True
        self.thread.start()

    def reset(self):
        self.lock.acquire()
        self.filter.reset()
        self.lock.release()

    def detect_state(self, sim):
        self.lock.acquire()
        self.filter.sensorUpdate(get_img())
        board = self.filter.getMostLikelyBoard()
        self.lock.release()
        return board
        
    def calculate_probs(self, img):
        self.lock.acquire()
        probs = calculate_probs(img)
        self.lock.release()
        return probs

    def sensorUpdate(self, img):
        self.lock.acquire()
        self.filter.sensorUpdate(img)
        self.lock.release()

    def modelUpdate(self, lastMove):
        self.lock.acquire()
        self.filter.modelUpdate(lastMove)
        self.lock.release()

    def _update_thread(self):
        while True:
            print("Updating filter")
            img = get_img()
            self.lock.acquire()
            self.filter.sensorUpdate(img)
            self.lock.release()
            print(self.filter.getMostLikelyBoard())
            self.rate.sleep()



def filter_test():
    detector = BayesFilter(ColorDetector())
    detector.board_belief[:,:,0] = 1.0
    detector.board_belief[:,:,1:] = 0.0
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img1.png")
    detector.modelUpdate([3,3,1])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img2.png")
    detector.modelUpdate([2,3,1])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img3.png")
    detector.modelUpdate([4,3,1])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img4.png")
    detector.modelUpdate([-1,-1,2])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img5.png")
    detector.modelUpdate([-1,-1,2])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img6.png")
    detector.modelUpdate([3,0,1])
    print(detector.board_belief)
    print("--------------")
    visualize_probs(detector.board_belief, path="img7.png")
    sys.exit(0)



if __name__=='__main__':
    print("In test mode.")
    rospy.init_node("filter_test")
    filter_test();
