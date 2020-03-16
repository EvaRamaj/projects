#!/usr/bin/env python

from edo_hanoi.srv import *
import rospy
import numpy as np

USE_NEURAL = True
if USE_NEURAL:
    import neural_detector
else:
    import vision_node


def handle_get_state(req):
    print("Received request: sim:=%s, tokens:=%s" % (req.sim, req.tokens))
    if USE_NEURAL:
        positions = neural_detector.detect_positions(req.sim, req.tokens)
    else:
        positions = vision_node.detect_positions(req.sim, req.tokens)
        positions = positions[:req.tokens]
        rest_tokens = [-1] * (8-req.tokens)
        positions.extend(rest_tokens)
    print("Sending positions: %s" % positions)
    return HanoiStateResponse(np.array(positions, dtype=np.int8))


def hanoi_state_server():
    rospy.init_node('hanoi_state_server')

    s = rospy.Service('/edo/edo_hanoi_services/get_hanoi_state', HanoiState, handle_get_state)

    print "Ready to get the Hanoi state."

    rospy.spin()


if __name__ == "__main__":
    hanoi_state_server()
