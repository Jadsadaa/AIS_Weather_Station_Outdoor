#ifndef NB_BC95_G_h
#define NB_BC95_G_h

#include <Arduino.h>

#define MODE_STRING 0
#define MODE_STRING_HEX 1
const char HEX_CHAR[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

typedef struct NB_SPLIT
{
  bool hasValue = false;
  String result[20] =
  {
    "","","","","",
    "","","","","",
    "","","","","",
    "","","","",""
  };
} nb_split_t;

typedef struct NB_SERVER
{
  String serverIP;
  String serverPort;
} nb_server_t;

typedef struct NB_SOCKET 
{
  bool created = false;// 
  unsigned char type;//DGRAM=UDP,STREAM=TCP
  unsigned char protocol;//UDP=17,TPC=6
  unsigned int port;//0-65535 
  unsigned char socket;//0-7 
  String serverIP;
  String serverPort;
} nb_socket_t;

typedef struct NB_BC95_G_RESP
{
    bool status;
    String data;
    String temp;
} nb_resp_t;

typedef struct NB_PING
{
  bool status;
  String addr;
  String ttl;
  String rtt;
} nb_ping_t;

typedef struct NB_SIGNAL
{
  String csq;
  String rssi;
  String ber;
} nb_signal_t;

typedef struct NB_SEND_UDP
{
  bool status;
  String send_str;
  unsigned char socket;
  unsigned int length;
} nb_udp_send_t;

typedef struct NB_RECEIVE_UDP
{
  unsigned char socket;
  String ip_address;
  unsigned int port;
  unsigned int length;
  String data;
  unsigned int remaining_length;
} nb_udp_receive_t;
 

class NB_BC95_G
{
  public:
    NB_BC95_G();
    
    bool debug = true;
    unsigned int delayBeforeCommand = 10;
    unsigned int delayAfterCommand = 10;
    
    nb_server_t myserver;
    nb_socket_t mysocket[7];
    
    void begin(unsigned int buadrate, unsigned int parity, unsigned int rx_pin, unsigned int tx_pin);

    /* Control Command Function */
    
    void setupDevice(String serverPort);

    bool attachNB(String serverPort);
    bool detachNB();
    bool getNBConnect();

    void createUDPSocket(String serverPort); 
    String getSocketResponse(); 
    

    nb_resp_t sendUDPMessage(unsigned char socket,String remote_addr,String remote_port,unsigned int len,String data_str,unsigned char send_mode);
    //nb_resp_t sendUDPFlag(unsigned char socket,String remote_addr,String remote_port,unsigned int len,String data_str,unsigned char send_mode);
    nb_resp_t closeUPDSocket(unsigned char socket);
 
    //nb_resp_t createTCPSocket();
    //nb_resp_t connectTCP(unsigned char socket,String remote_addr, String remote_port);                        
    //nb_resp_t sendTCPMessage(unsigned char socket,unsigned int len,char* data);
    //nb_resp_t closeTCPSocket(unsigned char socket);
 
    //nb_resp_t getListSocketMessage();
    nb_resp_t recieveCommand(unsigned char socket, unsigned int len);

    /* General Command Function */
      
    nb_resp_t testCommand();
    nb_resp_t rebootModule();
    nb_resp_t autoConnectOn();
    nb_resp_t autoConnectOff();
    nb_resp_t setFuncMode(unsigned char fun);

    nb_resp_t getAPN();
    nb_resp_t setAPN(String apn);

    nb_resp_t getNetworkStatus();
    nb_resp_t setNetworkStatus(unsigned char type);
    
    nb_resp_t getModuleStatus();
    nb_resp_t getFirmwareRevision();
    nb_signal_t getSignal();
    nb_resp_t getIMEI();
    nb_resp_t getIMSI();
    nb_resp_t getNCCID();
    
    nb_resp_t getCGATT(); 
    nb_resp_t setCGATT(unsigned char state);
    nb_resp_t getDeviceIP(); 
    nb_resp_t testPing(String serverIP);
  
    nb_resp_t sendCommand(String com_str, long timeout); 

    String subFromData(String data_str, String from_data);
    nb_split_t splitFromData(String data_str, String from_data);
    
  	char char2Byte(char c);
    String str2HexStr(String data_str);
    String hex2Str(String data_hex);

  private:
    nb_resp_t wait_nb_response(String wait_str, long timeout);
	  void myserialFlush();

  protected: 
};

#endif
