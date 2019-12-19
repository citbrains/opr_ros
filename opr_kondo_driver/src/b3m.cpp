#include    <boost/date_time.hpp>
#include    "opr_kondo_driver/KSerialPort.h"
#include    "opr_kondo_driver/b3m.hpp"

extern "C"
{
static KSerialPort port;
static short servo_rs_id_d[SERV_NUM]
    = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
unsigned short b3m_normal_mode[SERV_NUM];
unsigned short b3m_free_mode[SERV_NUM];
unsigned short b3m_control_mode[SERV_NUM];
unsigned short b3m_run_or_control[SERV_NUM];
unsigned short b3m_trajectory_normal[SERV_NUM];
unsigned short b3m_trajectory_even[SERV_NUM];
unsigned short b3m_gain_preset[SERV_NUM];
unsigned short b3m_deadband_width[SERV_NUM];
unsigned short b3m_control_kp0[SERV_NUM];
unsigned short b3m_control_kd0[SERV_NUM];
unsigned short b3m_control_ki0[SERV_NUM];
unsigned short b3m_control_static_friction0[SERV_NUM];
unsigned short b3m_control_dynamic_friction0[SERV_NUM];
unsigned short b3m_control_kp1[SERV_NUM];
unsigned short b3m_control_kd1[SERV_NUM];
unsigned short b3m_control_ki1[SERV_NUM];
unsigned short b3m_control_static_friction1[SERV_NUM];
unsigned short b3m_control_dynamic_friction1[SERV_NUM];
unsigned short b3m_control_kp2[SERV_NUM];
unsigned short b3m_control_kd2[SERV_NUM];
unsigned short b3m_control_ki2[SERV_NUM];
unsigned short b3m_control_static_friction2[SERV_NUM];
unsigned short b3m_control_dynamic_friction2[SERV_NUM];
unsigned short b3m_goal_time_slow[SERV_NUM];
signed short b3m_servo_offset[SERV_NUM];

void B3MInitializeVariable()
{
    for(int i=0; i<SERV_NUM; i++){
        b3m_normal_mode[i] = B3M_OPTIONS_RUN_NORMAL;
        b3m_free_mode[i] = B3M_OPTIONS_RUN_FREE;
        b3m_control_mode[i] = B3M_OPTIONS_CONTROL_POSITION;
        b3m_run_or_control[i] = b3m_free_mode[i-1] + b3m_control_mode[i-1];
        b3m_trajectory_normal[i]= B3M_OPTIONS_TRAJECTORY_NORMAL;
        b3m_trajectory_even[i] = B3M_OPTIONS_TRAJECTORY_1;
        b3m_gain_preset[i] = B3M_CONTROL_GAIN_PRESET_DEF;
        b3m_deadband_width[i] = 15;
        b3m_control_kp0[i] = 0;
        b3m_control_kd0[i] = 0;
        b3m_control_ki0[i] = 0;
        b3m_control_static_friction0[i] = 0;
        b3m_control_dynamic_friction0[i] = 0;
        b3m_control_kp1[i] = 0;
        b3m_control_kd1[i] = 0;
        b3m_control_ki1[i] = 0;
        b3m_control_static_friction1[i] = 0;
        b3m_control_dynamic_friction1[i] = 0;
        b3m_control_kp2[i] = 0;
        b3m_control_kd2[i] = 0;
        b3m_control_ki2[i] = 0;
        b3m_control_static_friction2[i] = 0;
        b3m_control_dynamic_friction2[i] = 0;
        b3m_goal_time_slow[i] = SERVO_B3M_DATA_GOAL_TIME_SLOW;
        b3m_servo_offset[i] = 0;
    }
}

// 通信ポートのオープン
int RSOpen( const char *portname )
{
    if( port.open(portname) ) {
        fprintf(stderr, "ERROR:Com port open error[%s]\n",portname );
        return -1;
    }
    port.setBaudRate(1000000);
    port.setParity(KSerialPort::KS_PARITY_NONE);
    return 0;
}

// 通信ポートのクローズ
void RSClose()
{
    port.close();
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボを移動させる
 *
 *  関数：int B3Move( char servoID, short sPos, unsigned short sTime )
 *  引数：
 *      char            servoID     サーボID
 *      short           sPos        移動位置
 *      unsigned short  sTime       移動時間
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3Move(char servoID, short sPos, unsigned short sTime)
{
    char sendbuf[28], readbuf[28];
    short sum;
    int ret;

    memset( sendbuf, 0x00, sizeof( sendbuf ));

    sendbuf[0] = (unsigned char)0x09;
    sendbuf[1] = (unsigned char)0x06;
    sendbuf[2] = (unsigned char)0x00;
    sendbuf[3] = (unsigned char)servoID;
    sendbuf[4] = (unsigned char)(sPos&0x00FF);
    sendbuf[5] = (unsigned char)((sPos&0xFF00)>>8);
    sendbuf[6] = (unsigned char)(sTime&0x00FF);
    sendbuf[7] = (unsigned char)((sTime&0xFF00)>>8);

    /*Check Sum*/
    sum = sendbuf[0];
    for(int i=1;i < 8;i++)
        sum += sendbuf[i];

    sendbuf[8] = (unsigned char)(sum&0x00FF);

    /*Send & Read Buffer*/
    boost::system::error_code err;
    ret = port.write_some( sendbuf, 9, err);

    return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボの軌道生成タイプの変更
 *
 *  関数：int B3Mode( char servoID , char mode)
 *  引数：
 *      char            servoID     サーボID
 *      char            mode        0:Normal    1:Even
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3Mode( char servoID,char mode)
{
    char sendbuf[28];
    short sum;
    int ret;

    memset( sendbuf, 0x00, sizeof( sendbuf ));

    sendbuf[0] = (unsigned char)0x08;
    sendbuf[1] = (unsigned char)0x04;
    sendbuf[2] = (unsigned char)0x00;
    sendbuf[3] = (unsigned char)servoID;
    sendbuf[4] = (unsigned char)0x01;
    sendbuf[5] = (unsigned char)0x29;
    sendbuf[6] = (unsigned char)mode;

    /*check sum*/
    sum = sendbuf[0];
    for(int i=1;i < 7;i++)
        sum += sendbuf[i];

    sendbuf[7] = (unsigned char)(sum&0x00FF);
    /*send & read buffer*/
    boost::system::error_code err;
    ret = port.write_some( sendbuf, 8, err);
    return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボの現在角度を取得する
 *
 *  関数：short B3GetAngle( char servoID, int *out_angle )
 *  引数：
 *      char            servoID     サーボID
 *      int             *out_angle  現在角度(0.01度 = 1)    
 *  戻り値：
 *      0
 *
 */
int B3MGetAngle( char servoID, int *out_angle )
{
    char    sendbuf[7];
    char    readbuf[8];
    unsigned char   sum;
    int             ret;
    unsigned long   len, readlen;
    short           angle;
    char            size = 2;

    //clear buf
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    //make packet
    sendbuf[0]  = (unsigned char)0x07;
    sendbuf[1]  = (unsigned char)0x03;
    sendbuf[2]  = (unsigned char)0x00;
    sendbuf[3]  = (unsigned char)servoID;
    sendbuf[4]  = (unsigned char)0x2C;
    sendbuf[5]  = (unsigned char)size;

    //calc CheckSum
    sum = sendbuf[0];
    for(int  i=1;i < 7;i++)
        sum += sendbuf[i];
        
    sendbuf[6] = (unsigned char)(sum&0x00FF);

    //Send buf
    boost::system::error_code err;
    ret = port.write_some( sendbuf, 7, err);

    //Read wait
    boost::this_thread::sleep(boost::posix_time::microseconds(250));

    //Read thread
    memset( readbuf, 0x00, sizeof( readbuf ));
    readlen = size + 5;

    len = 0;
    len = port.read( readbuf, sizeof( readbuf ), err);

    //check ReadData
    sum = readbuf[7];

    angle = ((readbuf[6]) | (readbuf[5] << 8));
    *out_angle = angle;

    return 0;
}

int B3MGetBurden( char servoID, int *out_burden ){
    return 0;
}

int B3MGetServoStatus( char servoID, ServoStatus *out_status )
{
    char    sendbuf[7];
    char    readbuf[8];
    unsigned char   sum;
    int             i;
    int             ret;
    unsigned long   len, readlen;
    short           burden;
    char            size    = 2;

    out_status->pos = 0;
    out_status->time_elapsed = 0;
    out_status->speed = 0;
    out_status->load = 0;
    out_status->temperature = 0;
    out_status->voltage = 0;

    memset( sendbuf, 0x00, sizeof( sendbuf ));

    sendbuf[0]  = (unsigned char)0x07;
    sendbuf[1]  = (unsigned char)0x03;
    sendbuf[2]  = (unsigned char)0x00;
    sendbuf[3]  = (unsigned char)servoID;
    sendbuf[4]  = (unsigned char)0x4A; // Specify input voltage addres
    sendbuf[5]  = (unsigned char)size;
    sum = sendbuf[0];
    for( i = 1; i < 7; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[6] = (unsigned char)(sum&0x00FF);
    boost::system::error_code err;
    ret = port.write_some(sendbuf, 7, err);
    if( ret < 7 ){
        printf("writeError\n");
        return -1;
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));

    memset( readbuf, 0x00, sizeof( readbuf ));
    readlen = size + 5;
    len = 0;

    len = port.read(readbuf, readlen, err);

    // check ReadDate
    sum = readbuf[7];

    out_status->voltage = ((readbuf[6]) | (readbuf[5] << 8)) / 1000.;

    return 0;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボのトルクをON/OFFする
 *
 *  関数：int B3MTorqueOnOff( KSerialPort &port, short sMode, char servoID )
 *  引数：
 *      KSerialPort     &port       通信ポートのハンドル
 *      short           sMode       1:トルクON
 *                                  0:トルクOFF
 *      char            servoID     サーボID
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3MTorqueOnOff( short sMode, char servoID )
{
    char    sendbuf[28];
    unsigned char   sum;
    int             i;
    int             ret;


    // バッファクリア
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    short mode = (sMode == 0) ? 0x02 : 0x00;
    // パケット作成
    sendbuf[0]  = (unsigned char)0x08;
    sendbuf[1]  = (unsigned char)0x04;
    sendbuf[2]  = (unsigned char)0x00;
    sendbuf[3]  = (unsigned char)servoID;
    sendbuf[4]  = (unsigned char)mode;
    sendbuf[5]  = (unsigned char)0x28;
    sendbuf[6]  = (unsigned char)0x01;
    
    // チェックサムの計算
    sum = sendbuf[0];
    for( i = 1; i < 7; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[7] = sum;                               // チェックサム
    // 送信
    boost::system::error_code err;
    ret = port.write_some(sendbuf, 8,err);

    return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボのトルクを設定する
 *
 *  関数：int B3MTorqueDown( KSerialPort port, char servoID )
 *  引数：
 *      KSerialPort         port        通信ポートのハンドル
 *      short           sMode       1:トルクON
 *                                  0:トルクOFF
 *      char            servoID     サーボID
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3MTorqueDown( char servoID )
{
    return 0;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボのトルクを一括で抑える
 *
 *  関数：int B3MTorqueALLDown( KSerialPort port, short sMode, char servoID )
 *  引数：
 *      KSerialPort         port        通信ポートのハンドル
 *      short           sMode       1:トルクON
 *                                  0:トルクOFF
 *      char            servoID     サーボID
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
void B3MTorqueALLDown( void )
{
    /*
    B3MTorqueDown( FOOT_ROLL_L);
    B3MTorqueDown( KNEE_L1);
    B3MTorqueDown( KNEE_L2);
    B3MTorqueDown( LEG_PITCH_L);
    B3MTorqueDown( LEG_ROLL_L);
    B3MTorqueDown( LEG_YAW_L);
    B3MTorqueDown( FOOT_ROLL_R);
    B3MTorqueDown( KNEE_R1);
    B3MTorqueDown( KNEE_R2);
    B3MTorqueDown( LEG_PITCH_R);
    B3MTorqueDown( LEG_ROLL_R);
    B3MTorqueDown( LEG_YAW_R);
    B3MTorqueDown( ARM_PITCH_L);
    B3MTorqueDown( ARM_ROLL_L);
    B3MTorqueDown( ARM_PITCH_R);
    B3MTorqueDown( ARM_ROLL_R);
    */
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボのトルクをまとめてON/OFFする
 *
 *  関数：int B3MTorqueOnOff( KSerialPort &port, short sMode )
 *  引数：
 *      KSerialPort     port        通信ポートのハンドル
 *      short           sMode       1:トルクON
 *                                  0:トルクOFF
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3MTorqueALLOnOff( short sMode )
{
    const int NUM_SERVO = 30;
    char    sendbuf[128];
    unsigned char   sum;
    int             i, servo_no;
    int             ret;

    // バッファクリア
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    // パケット作成
    short mode = (sMode == 0) ? 0x02 : 0x00;
    sendbuf[0]  = (unsigned char)(6+NUM_SERVO*3);
    sendbuf[1]  = (unsigned char)0x04;
    sendbuf[2]  = (unsigned char)0x00;
    for( servo_no = 1; servo_no <= NUM_SERVO; servo_no++){
        sendbuf[3+(servo_no-1)*3] = (unsigned char)servo_no;
        sendbuf[4+(servo_no-1)*3] = (unsigned char)mode;
        sendbuf[5+(servo_no-1)*3] = (unsigned char)0x00;
    }
    sendbuf[NUM_SERVO*3+3]  = (unsigned char)0x28;
    sendbuf[NUM_SERVO*3+4]  = (unsigned char)NUM_SERVO;

    // チェックサムの計算
    sum = sendbuf[0];
    for( i = 1; i <= NUM_SERVO*3+4; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[NUM_SERVO*3+5] = sum;                               // チェックサム
    
// 送信
    boost::system::error_code err;
    ret = port.write_some(sendbuf, NUM_SERVO*3+6,err);
    
    return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *  概要：サーボをまとめてリセットする
 *
 *  関数：int B3MAllReset( unsigned short sTime )
 *  引数：
 *      short           sTime       指定時間後にリセットを実行する
 *
 *  戻り値：
 *      0以上           成功
 *      0未満           エラー
 *
 */
int B3MAllReset( unsigned short sTime )
{
    char            sendbuf[128];
    unsigned char   sum;
    short           _id;
    short           len = 1;
    int             i, servo_no = 0;
    int             ret;
    
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
    // バッファクリア
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    // パケット作成
    sendbuf[0]  = (unsigned char)(5+SERV_NUM*(len+1));
    sendbuf[1]  = (unsigned char)0x05;
    sendbuf[2]  = (unsigned char)0x00;

    for( i = 0; i < SERV_NUM; i ++){
        _id = servo_rs_id_d[i];
        sendbuf[3+servo_no*(len+1)] = (unsigned char)_id;
        servo_no ++;
    }
    sendbuf[SERV_NUM*(len+1)+3]  = (unsigned char)sTime;

    sum = sendbuf[0];
    for( i = 1; i <= SERV_NUM*(len+1)+3; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[SERV_NUM*(len+1)+4] = (unsigned char)(sum&0x00FF);

    boost::system::error_code err;
    ret = port.write_some(sendbuf, SERV_NUM*(len+1)+5 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

    return ret;
}

int Write_Servo_B3M_All_2Kport(unsigned char addr, unsigned short *pdata, short len )
{
    char    sendbuf[128];
    unsigned char   sum;
    unsigned short  data;
    short           _id;
    int             i, servo_no = 0;
    int             ret = -1;
    
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
    // バッファクリア
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    // パケット作成
    sendbuf[0]  = (unsigned char)(6+SERV_NUM*(len+1));
    sendbuf[1]  = (unsigned char)0x04;
    sendbuf[2]  = (unsigned char)0x00;

    for( i = 0; i < SERV_NUM; i ++){
        _id = servo_rs_id_d[i];
        data = *(pdata + i);
        sendbuf[3+servo_no*(len+1)] = (unsigned char)_id;
        sendbuf[4+servo_no*(len+1)] = (unsigned char)(data&0x00FF);
        if (len == 2) {
            sendbuf[5+servo_no*(len+1)] = (unsigned char)((data&0xFF00)>>8);
        }
        servo_no ++;
    }
    sendbuf[SERV_NUM*(len+1)+3]  = (unsigned char)addr;             // フラグ
    sendbuf[SERV_NUM*(len+1)+4]  = (unsigned char)SERV_NUM;             // アドレス(0x24=36)

    sum = sendbuf[0];
    for( i = 1; i <= SERV_NUM*(len+1)+4; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[SERV_NUM*(len+1)+5] = sum;                              // チェックサム

    boost::system::error_code err;
    port.write_some(sendbuf, SERV_NUM*(len+1)+6 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

    return ret;
}

int Write_All_B3M_Position_or_Time( unsigned short *pdata, unsigned short *tdata, short len )
{
    char    sendbuf[128];
    unsigned char   sum;
    unsigned short  data;
    short           _id;
    int             i, servo_no = 0;
    int             ret = -1;
    
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
    // バッファクリア
    memset( sendbuf, 0x00, sizeof( sendbuf ));

    // パケット作成
    sendbuf[0]  = (unsigned char)(6+SERV_NUM*(len+1));
    sendbuf[1]  = (unsigned char)0x06;
    sendbuf[2]  = (unsigned char)0x00;

    for( i = 0; i < SERV_NUM; i ++){
        _id = servo_rs_id_d[i];
        data = *(pdata + i);
        sendbuf[3+servo_no*(len+1)] = (unsigned char)_id;
        sendbuf[4+servo_no*(len+1)] = (unsigned char)(data&0x00FF);
        if (len == 2) {
            sendbuf[5+servo_no*(len+1)] = (unsigned char)((data&0xFF00)>>8);
        }
        servo_no ++;
    }
    data = *tdata;
    sendbuf[SERV_NUM*(len+1)+3]  = (unsigned char)(data&0x00FF);                // set time
    sendbuf[SERV_NUM*(len+1)+4]  = (unsigned char)((data&0xFF00)>>8);

    sum = sendbuf[0];
    for( i = 1; i <= SERV_NUM*(len+1)+4; i++ ){
        sum += sendbuf[i];
    }
    sendbuf[SERV_NUM*(len+1)+5] = sum;                              // チェックサム

    boost::system::error_code err;
    port.write_some(sendbuf, SERV_NUM*(len+1)+6 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

    return ret;
}

void B3MInitializeSequence()
{
    float mode_sq_time;
    short mode_sq_start_prev = -1;
    short mode_sq_start = 0;
    bool flag_md_start_end = false;
    for(int i=0; i<SERV_NUM; i++){
        b3m_run_or_control[i] = b3m_free_mode[i-1] + b3m_control_mode[i-1];
    }

    while(!flag_md_start_end){
        if(mode_sq_start != mode_sq_start_prev){
            switch(mode_sq_start){
                    /***    servo_reset ***/
                case 0:
                    B3MAllReset(B3M_RESET_AFTER_TIME);
                    mode_sq_time = 0.06f;
                    break;

                case 1:
                    /***    servo_free_mode ***/
                    Write_Servo_B3M_All_2Kport( B3M_SERVO_SERVO_MODE, &b3m_free_mode[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 2:
                    Write_Servo_B3M_All_2Kport( B3M_SYSTEM_DEADBAND_WIDTH, &b3m_deadband_width[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;

                case 3:
                    /***    servo_control_mode  ***/
                    Write_Servo_B3M_All_2Kport( B3M_SERVO_SERVO_MODE, &b3m_run_or_control[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 4:
                    /***    servo_trajectory_even   ***/
                    Write_Servo_B3M_All_2Kport( B3M_SERVO_RUN_MODE, &b3m_trajectory_even[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 5:
                    /***    write_servo_normal_mode ***/
                    Write_Servo_B3M_All_2Kport( B3M_SERVO_SERVO_MODE, &b3m_normal_mode[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 6:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KP0, &b3m_control_kp0[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 7:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KD0, &b3m_control_kd0[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 8:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KI0, &b3m_control_ki0[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 9:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_STATIC_FRICTION0, &b3m_control_static_friction0[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 10:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_DYNAMIC_FRICTION0, &b3m_control_dynamic_friction0[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 11:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KP1, &b3m_control_kp1[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;

                case 12:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KD1, &b3m_control_kd1[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 13:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KI1, &b3m_control_ki1[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 14:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_STATIC_FRICTION1, &b3m_control_static_friction1[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 15:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_DYNAMIC_FRICTION1, &b3m_control_dynamic_friction1[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 16:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KP2, &b3m_control_kp2[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 17:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KD2, &b3m_control_kd2[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 18:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_KI2, &b3m_control_ki2[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 19:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_STATIC_FRICTION2, &b3m_control_static_friction2[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;
                    
                case 20:
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_DYNAMIC_FRICTION2, &b3m_control_dynamic_friction2[0], 2 );
                    mode_sq_time    =   0.02f;
                    break;

                case 21:
                    /***    write_servo_gain_change ***/
                    Write_Servo_B3M_All_2Kport( B3M_CONTROL_GAIN_PRESETNO, &b3m_gain_preset[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 22:
                    /***    write_servo_position    ***/
                    {
                        int i;
                        unsigned short position[SERV_NUM];
                        for(i = 0; i < SERV_NUM; i ++){
                            position[i] = b3m_servo_offset[i];
                        }
                        Write_All_B3M_Position_or_Time( (unsigned short *)&position[0], &b3m_goal_time_slow[0], 2 );
                    }
                    mode_sq_time    =   30.0f;
                    break;

                case 23:
                    /***    servo_trajectory_normal ***/
                    Write_Servo_B3M_All_2Kport( B3M_SERVO_RUN_MODE, &b3m_trajectory_normal[0], 1 );
                    mode_sq_time    =   0.02f;
                    break;

                case 24:
                    flag_md_start_end = true;
                    break;

                default:
                    break;
            
            }
        }

        mode_sq_start_prev = mode_sq_start;

        if ((!flag_md_start_end)&&(mode_sq_time < 0.001)){
            mode_sq_start++;
        }
        else{
            mode_sq_time -= 0.01;
        }
        usleep(1000);
    }
}

}// extern "C"
