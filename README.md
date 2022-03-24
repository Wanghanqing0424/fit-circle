# fit-circle
基于最小二乘法和RANSAC拟合圆
本项目使用了最小二乘法和RANSAC来对思岚雷达获取的数据进行处理，能较为有效的对球进行拟合
运行时需要配置ros环境，建议使用VSCODE配置ros节点，在工作空间编译src获得devel和lib包
bagfile为数据集文件，在终端进行rosbag play x.bag --loop即可
