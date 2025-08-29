#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import csv
import numpy as np
import rospy
import torch
from geometry_msgs.msg import PoseStamped

from class_poolingmodel_2nd_update_position import PoolingModel


# ---------- interpolation ----------
def upsample_traj(points: np.ndarray, factor: int = 10) -> np.ndarray:
    
    if len(points) < 2:                       
        return points.copy()
    n = points.shape[0]
    times = np.arange(n, dtype=float)

    new_times = np.linspace(0, n - 1, n * factor)

    x = np.interp(new_times, times, points[:, 0])
    y = np.interp(new_times, times, points[:, 1])
    return np.column_stack((x, y))



class TeleopUnconditionalExecutor:
    def __init__(self, model_path: str,
                 collection_duration: float = 8.0,  # Teleoperation duration
                 inference_rate: float = 80.0):


        rospy.init_node('teleop_unconditional_executor', anonymous=True)    

        # === data record ===
        self.initial_data   = []   
        self.predicted_data = []   
        self.real_time_data = []   

        # === 状态开关 ===
        self.collecting_initial   = True   # first 5 seconds
        self.model_prediction_active  = False   
        self.prediction_done      = False  

        # === counting ===
        self.collection_duration  = collection_duration
        self.start_time           = rospy.Time.now().to_sec()
        self.inference_rate       = inference_rate
        self.delta_t              = 1.0 / inference_rate
        self.reference            = None   # [N_init + N_pred*10, 2]
        self.reference_index      = 0
        self.max_reference_len    = 0

        # === 加载模型 ===
        self.model = PoolingModel(feature_size=2, hidden_size=32, action_dim=2)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        # === ROS ===
        
        self.target_pub = rospy.Publisher('/model_prediction',
                                          PoseStamped, queue_size=10)
        rospy.Subscriber('/arm_pose', PoseStamped, self.pose_callback)
        rospy.Timer(rospy.Duration(self.delta_t), self.inference_callback)
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("node start, operate manipulator in %.1f s ……",
                      self.collection_duration)

    
    def pose_callback(self, msg: PoseStamped):
        
        x = 20.0 * msg.pose.position.x
        y = 50.0 * msg.pose.position.y
        t = rospy.Time.now().to_sec()



        if self.collecting_initial:
            self.initial_data.append((t, x, y))
            print("pose_callback: t=%.2f, x=%.2f, y=%.2f" % (t, x, y))
        else:
            
            self.real_time_data.append((t, x, y))

   
    def inference_callback(self, _event):
        now = rospy.Time.now().to_sec()

        # stage 1: inital data collection
        if self.collecting_initial:
            if now - self.start_time >= self.collection_duration:
                self.collecting_initial = False
                self.model_prediction_active = True
                rospy.loginfo(" %d initial points sampled, model prediction starts.",
                              len(self.initial_data))
            return  

        # stage 2: model prediction
        if self.model_prediction_active and not self.prediction_done:
            init_len = len(self.initial_data)
            total_len = 1200
            if init_len >= total_len - 1:
                rospy.logwarn("initial points > =400, no need to predict.")
                self.model_prediction_active = False
                return

            
            reference = np.zeros((total_len, 2), dtype=float)
            for i, (_t, x, y) in enumerate(self.initial_data):
                reference[i] = [x, y]


            for i in range(init_len - 1, total_len - 1):
                inp = torch.tensor(reference, dtype=torch.float32)
                mu = self.model(inp.unsqueeze(0), delta_t=self.delta_t)\
                        .squeeze(0).detach().numpy()
                reference[i + 1] = mu[i]

          
            pred_segment = reference[init_len:total_len]
            pred_up      = upsample_traj(pred_segment, factor=10)

            self.reference        = np.concatenate(
                                        [reference[:init_len], pred_up], axis=0)
            self.reference_index  = init_len
            self.max_reference_len = len(self.reference)
            self.prediction_done   = True
            rospy.loginfo("inference done：initial %d points + predicted %d points (after interpolation %d), start publishing.",
                          init_len, total_len - init_len, self.max_reference_len)
            return  

        
        if self.prediction_done and self.reference_index < self.max_reference_len:
            xy = self.reference[self.reference_index]
            self.reference_index += 1
            self.publish_target_pose(xy)

            t = rospy.Time.now().to_sec()
            self.predicted_data.append((t, xy[0], xy[1]))

        elif self.prediction_done and self.reference_index >= self.max_reference_len:
            rospy.loginfo("all predicted points published")
            self.model_prediction_active = False  

   
    def publish_target_pose(self, xy):
        pose = PoseStamped()
        pose.header.stamp    = rospy.Time.now()
        pose.header.frame_id = "prediction"
        pose.pose.position.x = float(xy[0])
        pose.pose.position.y = float(xy[1])
        pose.pose.position.z = 0.487
        pose.pose.orientation.w = 1.0
        self.target_pub.publish(pose)

    
    def shutdown_hook(self):
        fname = "recorded_trajectory_data.csv"
        rospy.loginfo("save data to %s", fname)
        with open(fname, 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(["type", "time", "x", "y"])
            for typ, data in [("OBS", self.initial_data),
                              ("PRED", self.predicted_data),
                              ("REAL", self.real_time_data)]:
                for t, x, y in data:
                    wr.writerow([typ, t, x, y])
        rospy.loginfo("data saved。")



if __name__ == '__main__':
    try:
        TeleopUnconditionalExecutor(
            model_path="./pooling_model_v2_weights.pth",
            collection_duration=8.0,
            inference_rate=80.0
        )
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
