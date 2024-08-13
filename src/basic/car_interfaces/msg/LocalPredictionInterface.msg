# 局部轨迹预测话题
# 话题名称 loacal_prediction_data 
# 话题接口 car_interface/msg/LocalPredictionInterface.msg
float32     error_x
float32     error_y
float32     error_yaw
float32     error_speed
float64[]   x                       #途径点x
float64[]   y                       #途径点y
float64[]   yaw                     #途径点航向角角度
float64     timestamp               #数据时间戳
float32     speed                   #车辆速度
float32     angle                   #车辆角度
float32     process_time            #进程处理时间