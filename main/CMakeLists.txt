# idf_component_register(SRCS "main.c" "pid.c" "Kalman_filter.c" "mpu6050_v2.c" "PWM_write.c" "PWM_read.c" "Real-Time-Project.c"
#                     INCLUDE_DIRS ".")
set(SOURCES 
    main.c
    Kalman/kalman.c
    PID/pid.c
    MPU6050/mpu6050.c
    PWM/pwm_write.c
    PWM/pwm_read.c
)

set(INCLUDES 
    .
    Kalman
    PID
    MPU6050
    PWM
)

idf_component_register(SRCS ${SOURCES} INCLUDE_DIRS ${INCLUDES})
