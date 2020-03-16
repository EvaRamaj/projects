#!/usr/bin/env python2

from edo_connect4.srv import *
import rospy
import numpy as np
from connect4_detector import *
from connect4_filter import *
from player_detector import player_detector

rospy.init_node('connect4_state_server')
#here you can set other detectors
#detector = ColorDetector()
#detector = ColorDetector()
#detector = BayesFilter(ColorDetector())
#detector = ThreadedFilter(AverageFilter(ColorDetector()))
detector = ThreadedFilter(BayesFilter(ColorDetector()))

detectorSim = SimulationDetector()

def handle_get_state(req):
    print("Received Connect4State request: sim:=%s" % (req.sim))
    if req.sim:
        board = detectorSim.detect_state(req.sim)
    else:
        board = detector.detect_state(req.sim)
    board_as_list = []
    for c in range(board.shape[0]):
        for r in range(board.shape[1]):
            board_as_list.append(int(board[c,r]))
    return Connect4StateResponse(board_as_list)

def handle_model_update(req):
    print("Received ModelUpdate request: last move: %s,%s,%s" % (req.last_move_column, req.last_move_row, req.last_move_player))
    if isinstance(detector, Connect4Filter):
        detector.modelUpdate([req.last_move_column, req.last_move_row, req.last_move_player])
    else:
        print("Detector is not a filter.")
    return ModelUpdateResponse()

def handle_reset(req):
    print("Received Reset request")
    detector.reset()
    detectorSim.reset()
    return ResetResponse()

def handle_player_detector(req):
    print "Returning the player's color"
    player = player_detector()
    return PlayerDetectorResponse(player)


def connect4_state_server():
    s = rospy.Service('edo_connect4_services/get_connect4_state', Connect4State, handle_get_state)
    s = rospy.Service('edo_connect4_services/model_update', ModelUpdate, handle_model_update)
    s = rospy.Service('edo_connect4_services/reset', Reset, handle_reset)
    s = rospy.Service('edo_connect4_services/player_detector', PlayerDetector, handle_player_detector)

    print("Connect4 services ready.")

    rospy.spin()


if __name__ == "__main__":
    connect4_state_server()
