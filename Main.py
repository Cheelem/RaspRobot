# By LLG 李林根
# Autonomous Vehicle on Raspberry Pi Control
from ObjectDetectThread import ObjectDetectThread
from RobotControl import RobotObstacleAvoidControlThread
from queue import Queue
import argparse


def __main__():
    # construct the argument parser and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-g", "--graph", default="./graph",
                    help="path to input graph file")
    ap.add_argument("-c", "--confidence", type=float, default=.9,
                    help="confidence threshold")
    ap.add_argument("-d", "--display", type=int, default=1,
                    help="switch to display image on screen")
    args = vars(ap.parse_args())
    # message queue
    msg_queue = Queue()
    obj_thread = ObjectDetectThread(args, msg_queue, "car")
    control = RobotObstacleAvoidControlThread(msg_queue)
    obj_thread.start()
    control.start()

__main__()
