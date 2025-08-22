import pandas as pd
import matplotlib.pyplot as plt

# 读取轨迹数据
df_traj = pd.read_csv('trajectory.csv')
df_circle = pd.read_csv('circle_ref.csv')

# 绘图
plt.figure(figsize=(10, 10))

# 绘制理想圆（参考）
plt.plot(df_circle['cx'], df_circle['cy'], 'g--', label='Target Circle (r=200)', alpha=0.7)

# 绘制无人机轨迹
plt.plot(df_traj['x'], df_traj['y'], 'b-', label='UAV Trajectory', linewidth=2)

# 标出圆心
plt.plot(200, 0, 'ro', label='Orbit Center (200, 0)', markersize=10)

# 起点和终点
plt.plot(df_traj['x'].iloc[0], df_traj['y'].iloc[0], 'gs', label='Start', markersize=8)
plt.plot(df_traj['x'].iloc[-1], df_traj['y'].iloc[-1], 'ks', label='End', markersize=8)

# 设置标题和标签
plt.title('UAV Circular Orbit Simulation (R=200, Center=(200,0))', fontsize=14)
plt.xlabel('X (m)', fontsize=12)
plt.ylabel('Y (m)', fontsize=12)
plt.axis('equal')  # 保持纵横比相等
plt.grid(True, alpha=0.3)
plt.legend()

# 显示
plt.tight_layout()
plt.show()