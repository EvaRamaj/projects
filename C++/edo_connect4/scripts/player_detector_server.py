
from edo_connect4.srv import *
import rospy

def handle_player_detector():
    print "Returning the player's color"
    player = 0
    return PlayerDetectorResponse(player)

def player_detector_server():
    s = rospy.Service('/edo_connect4_services/player_detector', player_detector)

    print("player detector service is ready.")

    rospy.spin()

if __name__ == "__main__":
    player_detector_server()
