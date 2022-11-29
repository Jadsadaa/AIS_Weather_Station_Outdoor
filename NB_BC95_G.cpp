#include "NB_BC95_G.h"

HardwareSerial serial(1);

NB_BC95_G::NB_BC95_G() 
{ 
  
} 
void NB_BC95_G::begin(unsigned int buadrate, unsigned int parity = SERIAL_8N1, unsigned int rx_pin = 14, unsigned int tx_pin = 27)
{ 
  serial.begin(buadrate, parity, rx_pin, tx_pin);
  //serial.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN); 
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("### Set Begin NB_BC95_G library v1.0 ###")); 
}

void NB_BC95_G::setupDevice(String serverPort)
{
  //bool is_ok = true;
  //for(int i = 0; i < 10; i++)
  nb_resp_t R1 = getIMEI();
  nb_resp_t R2 = getNCCID();
  nb_resp_t R3 = getFirmwareRevision();
  bool R4 = attachNB(serverPort);
}

bool NB_BC95_G::attachNB(String serverPort)
{
  bool result = false;
  Serial.println("attachNB");
  if(!getNBConnect())
  { 
    for(int i=1;i<60;i+=1)
    {
      if (debug) {  
        Serial.print(F("# Connecting NB-IoT Network"));
        Serial.print("i:");
        Serial.println(i);
      }
      nb_resp_t R0 = getAPN();
      if(R0.status){
        int c = 0;
        nb_split_t data_items = splitFromData(R0.temp,"\n");
        if(data_items.hasValue){
          if(data_items.result[0].length() > 0) {
            String data_items2 = subFromData(data_items.result[0],"+CGDCONT:0,");
            if(data_items2.length() > 0) {
              nb_split_t data_items3 = splitFromData(data_items2,",");
              if(data_items3.hasValue) {
                if(data_items3.result[0].length() > 0) {
                  c++;
                  if(debug) {
                    Serial.print("PDP_type: ");
                    Serial.println(data_items3.result[0]);
                    Serial.print("APN: ");
                    Serial.println(data_items3.result[1]);
                  }
                }
              }
            }
          }
        }
        if(c==0){
          nb_resp_t res_setAPN = setAPN("internet"); 
        } 
      }
      nb_resp_t R1 = setFuncMode(1);
      nb_resp_t R2 = autoConnectOn();
      nb_resp_t R3 = setCGATT(1);
      delay(4000);
      //if(getNBConnect()){ 
        result=true; 
        if (debug) Serial.print(F("> Connected"));
        if(mysocket[0].created==false) { 
          createUDPSocket(serverPort); 
        } 
        break;
      
      //}
      Serial.print(F("."));
    }
  } 
  else {
      if(mysocket[0].created==false){
        createUDPSocket(serverPort);
      } 
      return true; 
  } 
  return result;
}
bool NB_BC95_G::detachNB()
{
  bool result = false;
  myserialFlush();
  if (debug) Serial.print(F("# Disconnecting NB-IoT Network"));
  nb_resp_t _res = setCGATT(0); 
  delay(1000);
  for(int i=1;i<60;i+=1)
  {
    Serial.print(F("."));
    if(!getNBConnect()) { 
      
      result=true; 
      break;
    } 
    else {
      _res = setCGATT(0); 
    }
  }
  if (debug) Serial.println(F("> Disconnected"));
  return result;
}


bool NB_BC95_G::getNBConnect()
{
  bool result = false;
  nb_resp_t _res = getCGATT();
  if(_res.status){
    String _dat = subFromData(_res.temp, "+CGATT:");
    if(_dat.indexOf("1") != -1)
    {
      result = true;
    }
  } 
  return result;
}

void NB_BC95_G::createUDPSocket(String serverPort)
{
  delay(delayBeforeCommand);  
  serial.print(F("AT+NSOCR=DGRAM,17,"));
  serial.println(serverPort + ",1");
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+NSOCR=DGRAM,17,")); 
  if(debug) Serial.println(serverPort + ",1"); 
  if(debug) Serial.print(F("Received Command: "));
  nb_resp_t _res = wait_nb_response(F("OK"), 10000); 
  //if(_res.status)
  {
    mysocket[0].created = true;
    mysocket[0].serverPort = serverPort;
  }
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println();  
  delay(delayAfterCommand);
}


nb_resp_t NB_BC95_G::sendUDPMessage(unsigned char socket,String remote_addr,String remote_port,unsigned int len,String data_str,unsigned char send_mode)
{
  delay(delayBeforeCommand); 
  if(!attachNB(remote_port))
  {
    if (debug) Serial.println("# >Disconnected");
    nb_resp_t  res_err;
    return res_err;
  }
  serial.print(F("AT+NSOST=")); 
  serial.print(socket); 
  serial.print("," + remote_addr);
  serial.print("," + remote_port);
  if(debug){ 
    Serial.println("=================================");
    Serial.print(F("Send Command: AT+NSOST=")); 
    Serial.print(socket); 
    Serial.print("," + remote_addr);
    Serial.print("," + remote_port);
  }
  
  if(send_mode == MODE_STRING_HEX) { 
    String data_con = str2HexStr(data_str);
    serial.print("," + String(data_con.length()/2)); 
    serial.print("," + data_con); 
    if(debug){
      Serial.print("," + String(data_con.length()/2));
      Serial.print("," + data_con);
    }
  }
  else {
    serial.print("," + String(len)); 
    serial.print("," + data_str); 
    if(debug){
      Serial.print("," + String(len)); 
      Serial.print("," + data_str);
    } 
  }
  serial.println();  
  Serial.println(); 
  nb_resp_t _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println();  
  delay(delayAfterCommand);
}

String NB_BC95_G::getSocketResponse()
{
  String result = "";
  while(true)
  {
    nb_resp_t _res = recieveCommand(1, 100);
    if(_res.status){
      nb_split_t data_items = splitFromData(_res.temp,"\n");
      if(data_items.hasValue > 0) {
        nb_split_t data_str = splitFromData(data_items.result[0],",");
        if(data_str.hasValue) {
          String data_hex = data_str.result[4];
          String data_byte = data_str.result[5];
          if(data_hex.length() > 0) {
            result += hex2Str(data_hex);
            if(data_byte == "0") break;
          }
          else {
            break;
          }
        }
      }
      else {
        break;
      }
    }
    else {
      break;
    }
  } 
  return result;
}

nb_resp_t NB_BC95_G::recieveCommand(unsigned char socket, unsigned int len)
{
  delay(delayBeforeCommand); 
  serial.print(F("AT+NSORF="));
  serial.print(socket);
  serial.println("," + String(len)); 
  if(debug) Serial.print(F("Send Command: AT+NSORF=")); 
  if(debug) Serial.print(socket); 
  if(debug) Serial.println("," + String(len)); 
  if(debug) Serial.print(F("Received Command: "));
  nb_resp_t _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println();  
  delay(delayAfterCommand);
  return _res;
}
 
nb_resp_t NB_BC95_G::testCommand()
{ 
  delay(delayBeforeCommand); 
  serial.println(F("AT"));
  if(debug) Serial.println(F("Send Command: AT")); 
  if(debug) Serial.print(F("Received Command: "));
  nb_resp_t _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println();  
  delay(delayAfterCommand);
  return _res; 
} 
nb_resp_t NB_BC95_G::rebootModule() //Reboot the UE
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+NRB"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+NRB"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 30000); 
  if(debug) Serial.print(_res.temp);   
  if(debug) Serial.println(); 
  delay(delayAfterCommand);
  return _res; 
} 
nb_resp_t NB_BC95_G::autoConnectOn() //Config Auto Connection On
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+NCONFIG=AUTOCONNECT,TRUE"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+NCONFIG=AUTOCONNECT,TRUE")); 
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println(); 
  delay(delayAfterCommand);    
  return _res; 
} 
nb_resp_t NB_BC95_G::autoConnectOff() //Config Auto Connection Of
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+NCONFIG=AUTOCONNECT,FALSE"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+NCONFIG=AUTOCONNECT,FALSE")); 
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);     
  return _res; 
}  
nb_resp_t NB_BC95_G::setFuncMode(unsigned char fun = 1) //Set UE Functionality
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.print(F("AT+CFUN="));
  serial.println(fun);
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+CFUN=")); 
  if(debug) Serial.println(fun); 
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println(); 
  delay(delayAfterCommand);    
  return _res; 
}   
 
nb_resp_t NB_BC95_G::getAPN() // Access Point Name
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CGDCONT?"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CGDCONT?"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();  
  delay(delayAfterCommand);
  return _res; 
}
nb_resp_t NB_BC95_G::setAPN(String apn) //Set Access Point Name
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.print(F("AT+CGDCONT=1,"));
  serial.println("\"" + apn + "\"");
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+CGDCONT=1,")); 
  if(debug) Serial.println("\"" + apn + "\"");
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);     
  return _res; 
}  

nb_resp_t NB_BC95_G::getNetworkStatus() // EPS Network Registration Status
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CEREG?"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CEREG?"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}
nb_resp_t NB_BC95_G::setNetworkStatus(unsigned char type) //Set Network Registration Status
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.print(F("AT+CEREG="));
  serial.println(type);
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+CEREG=")); 
  if(debug) Serial.println(type); 
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);     
  return _res; 
}  

nb_resp_t NB_BC95_G::getModuleStatus() // Query UE Statistics
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+NUESTATS"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+NUESTATS"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();  
  delay(delayAfterCommand);
  return _res; 
}

nb_resp_t NB_BC95_G::getFirmwareRevision() // Request Manufacturer Revision
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CGMR"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CGMR"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}
nb_signal_t NB_BC95_G::getSignal() // Get Signal Strength Indicator
{
  nb_signal_t _res;
  nb_resp_t _tmp;
  String tmp;
  int x = 0;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CSQ"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CSQ"));  
  if(debug) Serial.print(F("Received Command: "));
  _tmp = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_tmp.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  if(_tmp.temp.indexOf(F("+CSQ"))!=-1)
    {
      int index = _tmp.temp.indexOf(F(":"));
      int index2 = _tmp.temp.indexOf(F(","));
      tmp = _tmp.temp.substring(index+1,index2);
      if (tmp == F("99"))
      {
        _res.csq = F("N/A");
        _res.rssi = F("N/A");
      }
      else
      {
        _res.csq = tmp;
        x = tmp.toInt();
        x = (2*x)-113;
        _res.rssi = String(x);
      }
      _res.ber  = _tmp.temp.substring(index2+1);
       Serial.println("# Get CSQ Signal: csq= " + _res.csq + ", rssi= " + _res.rssi + ", ber= " +_res.ber);
    }
  return _res; 
}
nb_resp_t NB_BC95_G::getIMEI() // Request Product Serial Number
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CGSN=1")); 
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CGSN=1"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println();
  delay(delayAfterCommand);    
  return _res; 
} 
nb_resp_t NB_BC95_G::getIMSI() // Request International Mobile Subscriber Identity number
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CIMI"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CIMI"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000);
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}
nb_resp_t NB_BC95_G::getNCCID() // USIM Card Identification
{
  nb_resp_t _res;
  String _nccid = "";
  delay(delayBeforeCommand); 
  serial.println(F("AT+NCCID"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+NCCID"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  _nccid = _res.temp.substring(9,28);
  
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  _res.data = _nccid;
  return _res; 
}
nb_resp_t NB_BC95_G::getCGATT() // PS Attach or Detect
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CGATT?"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CGATT?"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}
nb_resp_t NB_BC95_G::setCGATT(unsigned char state) // PS Attach or Detect 1 attach, 0 detach
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.print(F("AT+CGATT="));
  serial.println(state);
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+CGATT="));  
  if(debug) Serial.println(state);  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}

nb_resp_t NB_BC95_G::getDeviceIP() // Device IP address the module
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(F("AT+CGPADDR"));
  if(debug) Serial.println("=================================");
  if(debug) Serial.println(F("Send Command: AT+CGPADDR"));  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000); 
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println(); 
  delay(delayAfterCommand); 
  return _res; 
}
nb_resp_t NB_BC95_G::testPing(String serverIP) // Test Ping
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.print(F("AT+NPING="));
  serial.println(serverIP);
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: AT+NPING="));  
  if(debug) Serial.println(serverIP);  
  if(debug) Serial.print(F("Received Command: "));
  _res = wait_nb_response(F("OK"), 10000);  
  if(debug) Serial.print(_res.temp);
  if(debug) Serial.println();
  delay(delayAfterCommand);  
  return _res; 
}

nb_resp_t NB_BC95_G::sendCommand(String com_str, long timeout = 10000)
{
  nb_resp_t _res;
  delay(delayBeforeCommand); 
  serial.println(com_str);
  if(debug) Serial.println("=================================");
  if(debug) Serial.print(F("Send Command: "));
  if(debug) Serial.println(com_str); 
  _res = wait_nb_response(F("OK"), timeout); 
  if(debug) Serial.print(F("Received Command: "));
  if(debug) Serial.print(_res.temp);  
  if(debug) Serial.println(); 
  delay(delayAfterCommand); 
  return _res;
}  



String NB_BC95_G::subFromData(String data_str, String from_data)
{
  String result = ""; 
  int lenFrom = from_data.length(); 
  int lenData = data_str.length();
  int index = data_str.indexOf(from_data);
  if(index != -1) {
    int endl = 0;
    for(int i = index + lenFrom; i < lenData; i++){
      endl = i;
      if(data_str[i] == '\n' || 
        data_str[i] == '\r' || 
        data_str[i] == '\0'){ 
        break;  
      } 
    }
    result = data_str.substring(index + lenFrom, endl); 
  }
  return result;
}

nb_split_t NB_BC95_G::splitFromData(String data_str, String from_data)
{
  nb_split_t _res;
  int len = data_str.length();
  int lenSplit = from_data.length();
  if(len <=0 || lenSplit <= 0) {
    if(debug) Serial.print(F("split from data error length")); 
  }
  int index = 0;
  if(len > 0) { 
    if(lenSplit == 1) { 
      for(int i=0;i<len;i++){  
        if(data_str[i] == from_data[0]){
          i += 1-1;// lenSplit-backByte
          if(index < 20) index++;
          continue;
        }
        _res.result[index] += data_str[i]; 
        _res.hasValue = true;
      }
    }
    if(lenSplit > 1) {
      int c = 0;
      for(int i=0;i<len;i++){  
        c = 0;
        for(int j=0;j<lenSplit;j++){
          if(data_str[i+j] == from_data[j]){
            c++;
          } 
        }
        if(c>=lenSplit){
          i += lenSplit-1;// lenSplit-backByte
          if(index < 20) index++;
          continue;
        }  
        _res.result[index] += data_str[i]; 
        _res.hasValue = true;
      }        
    }
  }  
  return _res;
}


nb_resp_t NB_BC95_G::wait_nb_response(String wait_str, long timeout = 10000)
{
  unsigned long pv_ok = millis();
  unsigned long current_ok = millis(); 
  bool flag_out = true;
  bool is_time_out = false;
  bool res = false; 
  
  String received = "";
   
  nb_resp_t _res;
  _res.data = "";
  _res.temp = "";

  while(flag_out)
  {
    if(serial.available() > 0)
    {
      received = serial.readStringUntil('\n');
      _res.temp += received + '\n';
      if(received.indexOf(wait_str)!=-1)
      {
        res = true;
        flag_out = false;
      }
      else if(received.indexOf(F("ERROR"))!=-1)
      {
        res = false;
        flag_out = false;
      }
    }
    current_ok = millis();
    if(abs(current_ok - pv_ok) >= timeout)
    {
      pv_ok = current_ok;
      res = false;
      flag_out = false; 
      is_time_out = true;
    }
  }
  _res.status = res;
  _res.data = received; 
  if(is_time_out)
  {
    if(debug) Serial.println("time out");
    if(debug) Serial.println(_res.temp);
    myserialFlush(); 
  }
  return _res;
}

char NB_BC95_G::char2Byte(char c)
{
  if((c>='0') && (c<='9'))
  {
    return (c-48);
  }
  else if((c>='A') && (c<='F'))
  {
    return (c-65) + 10;
  }
  else if((c>='a') && (c<='f'))
  {
    return (c-97) + 10;
  }
  return 0;
}
String NB_BC95_G::str2HexStr(String data_str)
{
  int len = data_str.length(); 
  String data_result = "";  
  for(unsigned int i = 0; i < len; i ++ ){
     char c = data_str[i]; 
     data_result += HEX_CHAR[(c & 0xF0) >> 4];
     data_result += HEX_CHAR[(c & 0x0F) >> 0];    
  } 
  return data_result;
}
String NB_BC95_G::hex2Str(String data_hex)
{
  int len = data_hex.length();
  char r , hi, lo; 
  String data_result = "";
  if(len > 0) {
     if(len % 2 == 0) {
        for(unsigned int i=0; i < len; i+=2) {
          r = (char2Byte(data_hex[i]) << 4) + char2Byte(data_hex[i+1]);
          data_result += r;  
        }
     }
  }
  return data_result;
} 
void NB_BC95_G::myserialFlush()
{
	delay(10);
  if(serial.available() > 0)
  {
    if(debug) Serial.print("Flush: ");
  	while(serial.available())
  	{
  		char x = serial.read();
      if(debug) Serial.print(x);
  	}
    if(debug) Serial.println();
  }
}
