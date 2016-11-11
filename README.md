# mpu_rpi

这个是一个写在树莓派上的mpu6050驱动
提取出加速度和角速度之后用他们来计算空间偏角


目录说明
--
  calc_angle.c/h    计算角度
  fifo.c/h          数据结构
  mpu.c/h           mpu的驱动
  Makefile          linux驱动模块makefile
  install.sh        安装模块的脚本
  --test
     test_qt         上位机的qt测试程序
     其他             下位机的测试程序
