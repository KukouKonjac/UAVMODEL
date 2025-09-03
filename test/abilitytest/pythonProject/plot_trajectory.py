import pandas as pd
import matplotlib.pyplot as plt

# 读取轨迹数据
df_traj = pd.read_csv('trajectory.csv') # 假设你的CSV文件已经包含了'type'列来区分UAV和目标
df_circle = pd.read_csv('circle_ref.csv')

# 分离无人机和目标的数据
uav_data = df_traj[df_traj['type'] == 'uav']
target_data = df_traj[df_traj['type'] == 'target']

# 绘图
plt.figure(figsize=(10, 10))

# 绘制无人机轨迹
plt.plot(uav_data['x'], uav_data['y'], 'b-', label='UAV Trajectory', linewidth=2)

# 绘制目标轨迹
plt.plot(target_data['x'], target_data['y'], 'r-', label='Target Trajectory', linewidth=2)

# 起点和终点
plt.plot(uav_data['x'].iloc[0], uav_data['y'].iloc[0], 'gs', label='UAV Start', markersize=8)
plt.plot(uav_data['x'].iloc[-1], uav_data['y'].iloc[-1], 'ks', label='UAV End', markersize=8)
plt.plot(target_data['x'].iloc[0], target_data['y'].iloc[0], 'g^', label='Target Start', markersize=8) # 使用三角形表示目标起点
plt.plot(target_data['x'].iloc[-1], target_data['y'].iloc[-1], 'k^', label='Target End', markersize=8) # 使用三角形表示目标终点

# 设置标题和标签
plt.title('UAV and Target Circular Orbit Simulation (R=200, Center=(200,0))', fontsize=14)
plt.xlabel('X (m)', fontsize=12)
plt.ylabel('Y (m)', fontsize=12)
plt.axis('equal')  # 保持纵横比相等
plt.grid(True, alpha=0.3)
plt.legend()

# 显示
plt.tight_layout()
plt.show()