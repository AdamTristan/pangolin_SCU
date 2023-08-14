# properties of the data frame
VisionMaxLen = 16  # 最大长度
VisionDataLen = 10  # 数据位长度

# the defination of the meaning of each byte
HEADER_BYTE = 0  # 帧头，为RC
X_BYTE = 2  # 坐标
Y_BYTE = 4  # 竞速赛或者过障碍不需要X，Y，则设置为0
TASK_BYTE = 6 # 最高位为1是竞速赛 最高为0是障碍赛；竞速赛分为：启动区到必达区1（1），在必达区1停下（2），转弯后修正（3），去必达区2（4），在必达区2停下（5），转弯后修正（6），去必达区3（7），在必达区3停下（8），转弯后修正（9），去必达区4（10），在必达区4停下（11），转弯后修正（12），去启动区（13），在启动区停下（14）；障碍赛分为启动区1到启动区2（1），启动区2停下（2），转向后调整（3），启动区2到启动区3（4），启动区2停下（5），转向后调整（6），启动区3到启动区4（7），启动区4停下（8），转向后调整（9），启动区4到启动区1（10），启动区1停下（11）
FLOAT_DATA_BYTE = 7
MOVE_ENABLE_BYTE = 11
# checksum byte should be the last byte of each frame
# this byte might be modified in the future since more bytes might be added
CHECKSUM_BYTE = 12
