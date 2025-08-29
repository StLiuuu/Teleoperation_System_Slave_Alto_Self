import rospy
import csv
import numpy as np
from geometry_msgs.msg import PoseStamped
import scipy.io
import torch

from class_poolingmodel_2nd_update_position import PoolingModel


def upsample_traj_to_3000(original_300_points):
    """
    interpolate the original 300 points to create a new trajectory with 3000 points.
    """
    # original_300_points: shape [300, 2]
    times = np.arange(300, dtype=float)  # [0,1,2,...,299]
    # new_times: shape [3000,2]
    new_times = np.linspace(0, 299, 3000)  # 0..299 均分 3000 份

    # interpolate x and y 
    x_original = original_300_points[:, 0]
    y_original = original_300_points[:, 1]

    x_upsampled = np.interp(new_times, times, x_original)
    y_upsampled = np.interp(new_times, times, y_original)

    #  concatenate x and y into a single array
    return np.column_stack((x_upsampled, y_upsampled))


class PthUnconditionalGenerator(object):
    def __init__(self, model_path):
        # ========== data record ==========
        self.initial_data = []    # first 100 points from .mat file
        self.predicted_data = []  # last 300~3000 points from model prediction
        self.real_time_data = []  # real data 

        # ========== flag ==========
        self.prediction_done = False
        self.reference = None         # whole trajectory (after interpolation)
        self.reference_index = 0      # current index in self.reference

        # ========== inference rate ==========
        self.inference_rate = 80.0    
        self.delta_t = 1.0 / self.inference_rate

        # ========== model load ==========
        self.model = PoolingModel(feature_size=2, hidden_size=32, action_dim=2)
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

        self.model_prediction_active = False  
        self.max_reference_len = 0 # for automatic stopping

        # ========== initalise ROS ==========
        rospy.init_node('trajectory_executor', anonymous=True)
        self.target_pub = rospy.Publisher('/model_prediction', PoseStamped, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/arm_pose', PoseStamped, self.pose_callback)

        
        self.timer = rospy.Timer(rospy.Duration(self.delta_t), self.inference_callback)

        # model prediction is initially disabled
        rospy.set_param("/use_model_prediction", False)

        # save CSV when module is shutdown
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("TrajectoryExecutor started.")

    def init_trajectory(self):
        
        # 1) 读出数据
        mat = scipy.io.loadmat('Demo_Drawing_Drawing.mat')
        data = np.concatenate((mat['s_seg_intp']['DataP'])[0, :], axis=0).reshape(5, 10, 400)
        pos_data = data[:, 1:3, :].swapaxes(1,2)  # shape [5, 400, 2]

        # 2) 数据增广 + 缩放
        scale_factors = np.reshape([0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4], (-1, 1, 1, 1))
        aug_pos_data = np.reshape(pos_data * scale_factors, (-1, pos_data.shape[-2], pos_data.shape[-1]))
        scaling = 100
        aug_pos_data *= scaling

        # 3) 把坐标中心化（减去第一个点）
        aug_pos_data_centered = aug_pos_data - aug_pos_data[:, :1, :]

        # 4) 从第一个样本中取前 100 个点
        init_traj = aug_pos_data_centered[1][:100]  # shape [100,2]

        rospy.loginfo("Publishing initial 100-point trajectory...")

        # 5) 逐点发布这 100 个点
        for i in range(len(init_traj)):
            xy = init_traj[i]
            self.publish_target_pose(xy, from_initial_traj=True)
            t = rospy.Time.now().to_sec()
            self.initial_data.append((t, xy[0], xy[1]))

            rospy.sleep(self.delta_t)

        # 6) 启用模型推理
        self.model_prediction_active = True
        rospy.set_param("/use_model_prediction", True)
        rospy.loginfo("Initial trajectory done. Model prediction enabled.")

    def pose_callback(self, msg):
        if not self.model_prediction_active:
            return  

        x = 20.0 * msg.pose.position.x
        y = 50.0 * msg.pose.position.y
        t = rospy.Time.now().to_sec()
        self.real_time_data.append((t, x, y))

    def inference_callback(self, event):
        """
        定时回调：如果 /use_model_prediction=true，则执行一次推理（只执行一次），并发布插值后轨迹。
        若插值后的轨迹已经在 self.reference 中，则按点发布。
        """
        if not rospy.get_param("/use_model_prediction", False):
            return  # 如果还没允许模型预测，直接 return
        
        if not self.model_prediction_active:
            return  # 如果模型预测还没激活，也直接 return

        if not self.prediction_done:
            # 第一次进来：做 400 步推理，其中前100点来自 initial_data，后300点来自模型
            time_steps_user = 100
            time_steps_total = 400
            reference = np.zeros((time_steps_total, 2))
            
            if len(self.real_time_data) < 100:
                rospy.logwarn("Not enough real-time data to start prediction. Need at least 100 points.")
                return      

            for i, (_, x, y) in enumerate(self.real_time_data):
                _,x, y = self.real_time_data[i]
                reference[i] = [x, y]

            # B) 后300点，用模型预测
            for i in range(time_steps_user - 1, time_steps_total - 1):
                inputs = torch.tensor(reference, dtype=torch.float32)  # shape [400,2]
                mu_preds = self.model.forward(inputs.unsqueeze(0), delta_t=self.delta_t).squeeze(0)  # [400,2]
                reference[i + 1] = mu_preds[i].detach().numpy()

            # 此时 reference.shape = [400,2], 含前100 + 后300
            # 我们要把后300点 (reference[100:400]) 变成 3000 点
            pred_300 = reference[100:400]  # 形状 [300,2]
            pred_3000 = upsample_traj_to_3000(pred_300)  # [3000,2]

            # 拼成新的 reference: [ 前100 (不插值) + 后3000 (插值后) ]
            self.reference  = np.concatenate([reference[:100], pred_3000], axis=0)  # shape [3100,2]

            self.reference_index = 100  # 从第 100 个开始往后发

            self.max_reference_len = len(self.reference)  # 3100

            self.prediction_done = True  # 标记预测已完成
            rospy.loginfo("Model prediction done. 300 -> 3000 upsample done. Now start publishing them.")
            return

        # 如果已经有了 self.reference，就一帧帧发布
        if self.reference_index < self.max_reference_len:
            xy = self.reference[self.reference_index]
            self.reference_index += 1
            self.publish_target_pose(xy, from_initial_traj=False)

            # record the time and predicted position
            t = rospy.Time.now().to_sec()
            self.predicted_data.append((t, xy[0], xy[1]))
        else:
            # if all points have been published, stop the model prediction
            rospy.loginfo("All points published. Stopping model prediction.")
            self.model_prediction_active = False
            rospy.set_param("/use_model_prediction", False)
            
    def publish_target_pose(self, xy, from_initial_traj=False):
        """
        发布 PoseStamped 消息到 /model_prediction
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "initial" if from_initial_traj else "prediction"

        pose_msg.pose.position.x = float(xy[0])
        pose_msg.pose.position.y = float(xy[1])
        pose_msg.pose.position.z = 0.487

        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self.target_pub.publish(pose_msg)

    def shutdown_hook(self):
        """
        节点关闭时，把数据存到 CSV
        """
        csv_filename = "recorded_trajectory_data.csv"
        rospy.loginfo("Saving data to %s...", csv_filename)

        with open(csv_filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["type", "time", "x", "y"])
            for (t, x, y) in self.initial_data:
                writer.writerow(["OBS", t, x, y])
            for (t, x, y) in self.predicted_data:
                writer.writerow(["PRED", t, x, y])
            for (t, x, y) in self.real_time_data:
                writer.writerow(["REAL", t, x, y])

        rospy.loginfo("Data saved successfully.")


# ========== main 函数 ==========
if __name__ == '__main__':
    try:
        model_path = './pooling_model_v2_weights.pth'

        executor = PthUnconditionalGenerator(model_path=model_path)
        executor.init_trajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass