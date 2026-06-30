std_msgs/Header header

bool diag_ebox_chg_mos_h_tem_limt    # 充电MOS高温限制标志
bool diag_ebox_chg_mos_h_tem_close   # 充电MOS高温关闭标志
bool diag_ebox_ebox_mos_h_tem_limt   # 电控箱MOS高温限制标志
bool diag_ebox_ebox_mos_h_tem_close  # 电控箱MOS高温关闭标志
bool diag_ebox_h_light_fault         # 前照灯故障
bool diag_ebox_b_light_fault         # 制动灯故障
bool diag_ebox_l_light_fault         # 左转向灯故障
bool diag_ebox_r_light_fault         # 右转向灯故障

float32 mcu_48v_voltage              # 驱动MCU电压 (V)
float32 mcu_current                  # 驱动MCU电流 (A)
float32 ebox_current                 # 电控箱总电流 (A)
float32 cus_48v_voltage              # 客户48V口电压 (V)
float32 cus_24v_voltage              # 客户24V口电压 (V)
float32 cus_12v_voltage              # 客户12V口电压 (V)
float32 bat_48v_voltage              # 电池输入端电压 (V)
float32 ebox_mos_tem                 # 电控箱MOS管温度 (℃)
float32 chg_mos_tem                  # 充电MOS管温度 (℃)