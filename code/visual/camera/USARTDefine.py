# properties of the data frame
VisionMaxLen = 16  # ��󳤶�
VisionDataLen = 10  # ����λ����

# the defination of the meaning of each byte
HEADER_BYTE = 0  # ֡ͷ��ΪRC
X_BYTE = 2  # ����
Y_BYTE = 4  # ���������߹��ϰ�����ҪX��Y��������Ϊ0
TASK_BYTE = 6 # ���λΪ1�Ǿ����� ���Ϊ0���ϰ�������������Ϊ�����������ش���1��1�����ڱش���1ͣ�£�2����ת���������3����ȥ�ش���2��4�����ڱش���2ͣ�£�5����ת���������6����ȥ�ش���3��7�����ڱش���3ͣ�£�8����ת���������9����ȥ�ش���4��10�����ڱش���4ͣ�£�11����ת���������12����ȥ��������13������������ͣ�£�14�����ϰ�����Ϊ������1��������2��1����������2ͣ�£�2����ת��������3����������2��������3��4����������2ͣ�£�5����ת��������6����������3��������4��7����������4ͣ�£�8����ת��������9����������4��������1��10����������1ͣ�£�11��
FLOAT_DATA_BYTE = 7
MOVE_ENABLE_BYTE = 11
# checksum byte should be the last byte of each frame
# this byte might be modified in the future since more bytes might be added
CHECKSUM_BYTE = 12
