#读取输出npy文件
import numpy as np

data=np.load("/home/g/graspness_implementation/logs/infer/scene_0005/kinect/0000.npy")
#输出文件
print(data)

# 查看数据的形状和类型
print("Shape:", data.shape)
print("Data type:", data.dtype)
# 打印前 5 行数据
print(data[:5])

# 打印特定行和列，例如第 1 行的第 3 列
print(data[0, 2])

# 打印第 5 列的所有值
print(data[:, 4])
