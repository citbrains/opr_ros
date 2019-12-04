#include    <boost/date_time.hpp>
//#include	<KSerialPort.h>
#include	"KSerialPort.h"
#include	"b3m.h"
#include	"var.h"

extern "C"
{

extern int  servo_offset[SERV_NUM];

static KSerialPort port;
static	short servo_rs_id_d[ACCELITE_SERV_NUM]
	= {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};

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
 *	概要：サーボを移動させる
 *
 *	関数：int B3Move( char servoID, short sPos, unsigned short sTime )
 *	引数：
 *		char			servoID		サーボID
 *		short			sPos		移動位置
 *		unsigned short	sTime		移動時間
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
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
 *	概要：サーボの軌道生成タイプの変更
 *
 *	関数：int B3Mode( char servoID , char mode)
 *	引数：
 *		char			servoID		サーボID
 *		char			mode		0:Normal	1:Even
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
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
 *	概要：サーボの現在角度を取得する
 *
 *	関数：short B3GetAngle( char servoID, int *out_angle )
 *	引数：
 *		char			servoID		サーボID
 *		int				*out_angle	現在角度(0.01度 = 1)	
 *	戻り値：
 *		0
 *
 */
int B3MGetAngle( char servoID, int *out_angle )
{
	char	sendbuf[7];
	char	readbuf[8];
	unsigned char	sum;
	int				ret;
	unsigned long	len, readlen;
	short			angle;
	char			size = 2;

	//clear buf
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	//make packet
	sendbuf[0]	= (unsigned char)0x07;
	sendbuf[1]	= (unsigned char)0x03;
	sendbuf[2]	= (unsigned char)0x00;
	sendbuf[3]	= (unsigned char)servoID;
	sendbuf[4]	= (unsigned char)0x2C;
	sendbuf[5]	= (unsigned char)size;

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
	char			size	= 2;

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
 *	概要：サーボのトルクをON/OFFする
 *
 *	関数：int B3MTorqueOnOff( KSerialPort &port, short sMode, char servoID )
 *	引数：
 *		KSerialPort		&port		通信ポートのハンドル
 *		short			sMode		1:トルクON
 *									0:トルクOFF
 *		char			servoID		サーボID
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 *
 */
int B3MTorqueOnOff( short sMode, char servoID )
{
	char	sendbuf[28];
	unsigned char	sum;
	int				i;
	int				ret;


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
	sendbuf[7] = sum;								// チェックサム
	// 送信
	boost::system::error_code err;
	ret = port.write_some(sendbuf, 8,err);

	return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *	概要：サーボのトルクを設定する
 *
 *	関数：int B3MTorqueDown( KSerialPort port, char servoID )
 *	引数：
 *		KSerialPort			port		通信ポートのハンドル
 *		short			sMode		1:トルクON
 *									0:トルクOFF
 *		char			servoID		サーボID
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 *
 */
int B3MTorqueDown( char servoID )
{
	return 0;
}

/*----------------------------------------------------------------------------*/
/*
 *	概要：サーボのトルクを一括で抑える
 *
 *	関数：int B3MTorqueALLDown( KSerialPort port, short sMode, char servoID )
 *	引数：
 *		KSerialPort			port		通信ポートのハンドル
 *		short			sMode		1:トルクON
 *									0:トルクOFF
 *		char			servoID		サーボID
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 *
 */
void B3MTorqueALLDown( void )
{
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
}

/*----------------------------------------------------------------------------*/
/*
 *	概要：サーボのトルクをまとめてON/OFFする
 *
 *	関数：int B3MTorqueOnOff( KSerialPort &port, short sMode )
 *	引数：
 *		KSerialPort		port		通信ポートのハンドル
 *		short			sMode		1:トルクON
 *									0:トルクOFF
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 *
 */
int B3MTorqueALLOnOff( short sMode )
{
	const int NUM_SERVO = 30;
	char	sendbuf[128];
	unsigned char	sum;
	int				i, servo_no;
	int				ret;

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
	sendbuf[NUM_SERVO*3+5] = sum;								// チェックサム
	
// 送信
	boost::system::error_code err;
	ret = port.write_some(sendbuf, NUM_SERVO*3+6,err);
	
	return ret;
}

/*----------------------------------------------------------------------------*/
/*
 *	概要：サーボをまとめてリセットする
 *
 *	関数：int B3MAllReset( unsigned short sTime )
 *	引数：
 *		short			sTime		指定時間後にリセットを実行する
 *
 *	戻り値：
 *		0以上			成功
 *		0未満			エラー
 *
 */
int B3MAllReset( unsigned short sTime )
{
	char			sendbuf[128];
	unsigned char	sum;
	short			_id;
	short			len = 1;
	int				i, servo_no = 0;
	int				ret;
	
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)(5+ACCELITE_SERV_NUM*(len+1));
	sendbuf[1]  = (unsigned char)0x05;
	sendbuf[2]  = (unsigned char)0x00;

	for( i = 0; i < ACCELITE_SERV_NUM; i ++){
		_id = servo_rs_id_d[i];
		sendbuf[3+servo_no*(len+1)] = (unsigned char)_id;
		servo_no ++;
	}
	sendbuf[ACCELITE_SERV_NUM*(len+1)+3]  = (unsigned char)sTime;

	sum = sendbuf[0];
	for( i = 1; i <= ACCELITE_SERV_NUM*(len+1)+3; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[ACCELITE_SERV_NUM*(len+1)+4] = (unsigned char)(sum&0x00FF);

	boost::system::error_code err;
	ret = port.write_some(sendbuf, ACCELITE_SERV_NUM*(len+1)+5 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

	return ret;
}

int Write_Servo_B3M_All_2Kport(unsigned char addr, unsigned short *pdata, short len )
{
	char	sendbuf[128];
	unsigned char	sum;
	unsigned short	data;
	short			_id;
	int				i, servo_no = 0;
	int				ret = -1;
	
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)(6+ACCELITE_SERV_NUM*(len+1));
	sendbuf[1]  = (unsigned char)0x04;
	sendbuf[2]  = (unsigned char)0x00;

	for( i = 0; i < ACCELITE_SERV_NUM; i ++){
		_id = servo_rs_id_d[i];
		data = *(pdata + i);
		sendbuf[3+servo_no*(len+1)] = (unsigned char)_id;
		sendbuf[4+servo_no*(len+1)] = (unsigned char)(data&0x00FF);
		if (len == 2) {
			sendbuf[5+servo_no*(len+1)] = (unsigned char)((data&0xFF00)>>8);
		}
		servo_no ++;
	}
	sendbuf[ACCELITE_SERV_NUM*(len+1)+3]  = (unsigned char)addr;				// フラグ
	sendbuf[ACCELITE_SERV_NUM*(len+1)+4]  = (unsigned char)ACCELITE_SERV_NUM;				// アドレス(0x24=36)

	sum = sendbuf[0];
	for( i = 1; i <= ACCELITE_SERV_NUM*(len+1)+4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[ACCELITE_SERV_NUM*(len+1)+5] = sum;								// チェックサム

	boost::system::error_code err;
	port.write_some(sendbuf, ACCELITE_SERV_NUM*(len+1)+6 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

	return ret;
}

int Write_All_B3M_Position_or_Time( unsigned short *pdata, unsigned short *tdata, short len )
{
	char	sendbuf[128];
	unsigned char	sum;
	unsigned short	data;
	short			_id;
	int				i, servo_no = 0;
	int				ret = -1;
	
    boost::posix_time::ptime t1 = boost::posix_time::microsec_clock::local_time(); 
	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)(6+ACCELITE_SERV_NUM*(len+1));
	sendbuf[1]  = (unsigned char)0x06;
	sendbuf[2]  = (unsigned char)0x00;

	for( i = 0; i < ACCELITE_SERV_NUM; i ++){
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
	sendbuf[ACCELITE_SERV_NUM*(len+1)+3]  = (unsigned char)(data&0x00FF);				// set time
	sendbuf[ACCELITE_SERV_NUM*(len+1)+4]  = (unsigned char)((data&0xFF00)>>8);

	sum = sendbuf[0];
	for( i = 1; i <= ACCELITE_SERV_NUM*(len+1)+4; i++ ){
		sum += sendbuf[i];
	}
	sendbuf[ACCELITE_SERV_NUM*(len+1)+5] = sum;								// チェックサム

	boost::system::error_code err;
	port.write_some(sendbuf, ACCELITE_SERV_NUM*(len+1)+6 ,err);
    boost::posix_time::ptime t2 = boost::posix_time::microsec_clock::local_time(); 
    boost::posix_time::time_duration diff = t2 - t1;
    //std::cout << "send servo : " << diff.total_microseconds() << std::endl;

	return ret;
}

}// extern "C"
