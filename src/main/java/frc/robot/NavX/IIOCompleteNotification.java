package frc.robot.NavX;

interface IIOCompleteNotification {
    class BoardState {
        public byte  op_status;
        public short sensor_status;
        public byte  cal_status;
        public byte  selftest_status;
        public short capability_flags;
        public byte  update_rate_hz;
        public short accel_fsr_g;
        public short gyro_fsr_dps;
    }
    void setYawPitchRoll(IMUProtocol.YPRUpdate yprupdate, long sensor_timestamp);
    void setAHRSData(AHRSProtocol.AHRSUpdate ahrs_update, long sensor_timestamp);
    void setAHRSPosData(AHRSProtocol.AHRSPosUpdate ahrs_update, long sensor_timestamp);
    void setRawData(IMUProtocol.GyroUpdate raw_data_update, long sensor_timestamp);
    void setBoardID(AHRSProtocol.BoardID board_id);
    void setBoardState( BoardState board_state, boolean update_board_status);
    void yawResetComplete();
    void disconnectDetected();
    void connectDetected();
}
