#!/usr/bin/env python3
import rospy
from coms.srv import MergeMap, MergeMapResponse, MergeMapRequest, TriggerMerge, TriggerMergeResponse, TriggerMergeRequest
from nav_msgs.srv import GetMap, GetMapResponse, GetMapRequest
from nav_msgs.msg import OccupancyGrid
from mapmerge.service import mapmerge_pipeline
from mapmerge.keypoint_merge import orb_mapmerge
from mapmerge.ros_utils import pgm_to_numpy, numpy_to_ros, ros_to_numpy
import numpy as np
import rosbag
from mapmerge.constants import *


class MergeHandler:
    def __init__(self):
        # load rospy parameters
        self.robot_name = rospy.get_namespace()
        self.robot_name = self.robot_name[1:]
        self.robot_name = self.robot_name[:-1]
        self.starting_map_path = rospy.get_param('starting_map', '')
        self.logging = bool(rospy.get_param('logging', 0))
        self.debug = bool(rospy.get_param('debug', 1))

        # init ros
        rospy.init_node("mergeHandler", anonymous=True)
        self.merge_service = rospy.Service("merge", MergeMap, self.merge_cb) # depreciated
        self.get_map_service = rospy.Service("get_map", GetMap, self.serve_map)
        self.trigger_merge = rospy.Service("trigger_merge", TriggerMerge, self.trigger_merge_cb)
        self.map_publisher = rospy.Publisher("merged_map", OccupancyGrid, queue_size=10)
        self.gmapping_subscriber = rospy.Subscriber("map", OccupancyGrid, self.gmapping_cb)
        self.rate = rospy.Rate(10)  # 10hz
        self.busy = False

        # logging
        if self.logging:
            self.bag = rosbag.Bag(f'{self.robot_name}_log.bag', 'w')

        self.has_merged = False
        self.map_info = None

        # map data
        self.seq = 0
        self.latest_map = np.array([])
        self.inital_shape = (0, 0)
        if self.starting_map_path != "":
            self.latest_map = pgm_to_numpy(self.starting_map_path)
            self.inital_shape = self.latest_map.shape
            print(f"starting with inital map of shape {self.inital_shape}")

    def run(self) -> None:
        rospy.loginfo(f'{self.robot_name} merge handler node starting')
        while not rospy.is_shutdown():
            if self.latest_map.any():
                self.publish_latest()
            self.rate.sleep()
        rospy.loginfo(f'{self.robot_name} merge handler node shutting down')
        if self.logging:
            self.bag.close()

    def publish_latest(self) -> None:
        occ = self.create_occupancy_msg(self.seq)
        self.seq += 1
        self.map_publisher.publish(occ)

    def gmapping_cb(self, msg: OccupancyGrid) -> None:
        """
        callback for gmapping,
        TODO gmapping:
            increase map update speed
            set map transform to remain the same after merges.
        """
        if self.busy:
          return

        if self.logging:
            self.bag.write('map', msg)

        self.map_header = msg.header
        if self.map_info is None:  # try not rewriting headers (pose estimate)
            self.map_info = msg.info
        # breakpoint()
        MergeHandler.parse_and_save(msg, self.latest_map)
        # unpack gmapping
        new_map = ros_to_numpy(msg.data).reshape(-1,msg.info.width)
        #if(self.robot_name == "Khepera_2"):          
        #  new_map.fill(UNKNOWN)
        self.merge_map(new_map, gmap=True)
        # print("merge")

    @staticmethod
    def parse_and_save(msg, old_map):
        """
        takes in an occupancy grid and saves it as an npy file
        """
        new_map = ros_to_numpy(msg.data).reshape(-1, msg.info.width)
        # create this directory if you don't already have
        with open(f'/root/catkin_ws/src/coms/coms/test1/local{msg.header.seq}.npy', 'wb') as f:
            np.save(f, old_map)
        with open(f'/root/catkin_ws/src/coms/coms/test1/foriegn{msg.header.seq}.npy', 'wb') as f:
            np.save(f, new_map)

    # depreciated
    def merge_cb(self, req: MergeMapRequest) -> MergeMapResponse:
        new_map = ros_to_numpy(req.map.data).reshape(-1, req.map.info.width)
        return self.merge_map(new_map)

    def trigger_merge_cb(self, req: TriggerMergeRequest) -> TriggerMergeResponse:
        """
        Gonna just do a ros merge for now:
        """
        self.busy=True
        name = req.robot_id
        new_map = self.get_map(name)

        with open(f'local{1}.npy', 'wb') as f:
            np.save(f, self.latest_map)
        with open(f'other{1}.npy', 'wb') as f:
            np.save(f, new_map)
        result = self.merge_map(new_map)
        self.busy=False
        return result

    def merge_map(self, new_map: np.array([]), gmap=False) -> bool:
        """
        merges a map.
        configurable number of retries and error catching
        """

        if not self.latest_map.any():
            self.latest_map = new_map
            return True

        try:
            # print(new_map.shape)
            # print(self.latest_map.shape)
            if not self.has_merged and gmap:
                self.latest_map = new_map
            else:
                self.latest_map = mapmerge_pipeline(new_map, self.latest_map)
                self.has_merged = True
            return True
        except Exception as e:
            rospy.logerr(f"Could not merge maps: {e}")
        return False

    def get_map(self, name: str) -> np.array([]):
        """
        gets the latest map from map service
        """
        if name[0] != '/':
            name = '/' + name

        rospy.wait_for_service(name + '/get_map')
        while 1:
            try:
                map_service = rospy.ServiceProxy(name + '/get_map', GetMap)
                resp = map_service()
                return ros_to_numpy(resp.map.data).reshape(-1, resp.map.info.width)
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" % e)
            self.rate.sleep()

    def serve_map(self, _) -> GetMapResponse:
        """
        serves the latest map
        """
        occ = self.create_occupancy_msg(self.seq)
        return GetMapResponse(occ)

    def create_occupancy_msg(self, seq: int) -> OccupancyGrid:
        occ = OccupancyGrid()
        # header
        # occ.header.seq = seq
        # occ.header.frame_id = "tb3_"+self.robot_name[-1]+"/map"
        # occ.header.stamp = rospy.Time.now()

        # # metadata
        # occ.info.resolution = 0.05000000074505806
        # occ.info.origin.orientation.w = 1.0
        occ.header = self.map_header
        occ.info = self.map_info
        occ.info.height = self.latest_map.shape[0]
        occ.info.width = self.latest_map.shape[1]

        # data
        occ.data = tuple(numpy_to_ros(self.latest_map.flatten()))
        return occ


if __name__ == "__main__":
    MH = MergeHandler()
    MH.run()
