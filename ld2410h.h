#include "esphome.h"
#include <sstream>

class ld2410 : public Component, public UARTDevice
{
public:
    ld2410(UARTComponent *parent) : UARTDevice(parent) {}

    Sensor *mov_distance_sensor = new Sensor();
    Sensor *occ_distance_sensor = new Sensor();
    Sensor *detect_distance_sensor = new Sensor();
    Sensor *en_mov_0 = new Sensor();
    Sensor *en_mov_1 = new Sensor();
    Sensor *en_mov_2 = new Sensor();
    Sensor *en_mov_3 = new Sensor();
    Sensor *en_mov_4 = new Sensor();
    Sensor *en_mov_5 = new Sensor();
    Sensor *en_mov_6 = new Sensor();
    Sensor *en_mov_7 = new Sensor();
    Sensor *en_mov_8 = new Sensor();
    Sensor *en_occ_0 = new Sensor();
    Sensor *en_occ_1 = new Sensor();
    Sensor *en_occ_2 = new Sensor();
    Sensor *en_occ_3 = new Sensor();
    Sensor *en_occ_4 = new Sensor();
    Sensor *en_occ_5 = new Sensor();
    Sensor *en_occ_6 = new Sensor();
    Sensor *en_occ_7 = new Sensor();
    Sensor *en_occ_8 = new Sensor();

    std::vector<uint8_t> bytes;
    std::vector<uint8_t> header_parameter = {0xFD, 0xFC, 0xFB, 0xFA, 0x1C, 0x00, 0x61, 0x01, 0x00, 0x00, 0xAA};
    std::vector<uint8_t> header_data = {0xF4, 0xF3, 0xF2, 0xF1, 0x0D, 0x00};
    std::vector<uint8_t> header_data_debug = {0xF4, 0xF3, 0xF2, 0xF1, 0x23, 0x00};
//调试用    std::vector<uint8_t> header_feedback = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00};


    bool HeaderMatched(std::vector<uint8_t> bytes, std::vector<uint8_t> header)
    {
        if (bytes[0] != header[0] || bytes[1] != header[1] || bytes[2] != header[2] || bytes[3] != header[3])
        {
            return false;
        }
        return bytes[4] == header[4] && bytes[5] == header[5];	
	}

    void setup() override 
    {
    // nothing to do here
    }
    
    void loop() override
    {
        static int data_count = 0;
        static bool presence_state = true;  //无人时不监控，节省运算量

        while (available() > 0)  // && (id(run_rta).state || id(config_mode).state)
        {
            bytes.push_back(read());
            if (bytes.size() < 6 + 3)    //多取3位，用于判断是否有人
            {
                continue;
            }

            if ((HeaderMatched(bytes, header_data) || HeaderMatched(bytes, header_data_debug)) && id(run_rta).state)
            {
                if (bytes[8])
                {
                    if (bytes.size() < 6 + bytes[4])
                    {
                        continue;
                    }

                    data_count++;
                    presence_state = true;
                    if (data_count > id(update_interval).state)
                    {
                        data_count = 0;
                        LDDATAUnion lddataUnion;
                        std::copy(bytes.begin() + 6, (bytes.begin() + 6 + bytes[4]), lddataUnion.bytes);
                        switch (lddataUnion.lddata.target_state)
                        {
                            case 1:
                                id(target_state).publish_state("mov");
                                break;
                            case 2:
                                id(target_state).publish_state("occ");
                                break;
                            case 3:
                                id(target_state).publish_state("mov & occ");
                                break;
                        }
                        mov_distance_sensor->publish_state(lddataUnion.lddata.mov_distance1 + lddataUnion.lddata.mov_distance2 * 256);
                        occ_distance_sensor->publish_state(lddataUnion.lddata.occ_distance1 + lddataUnion.lddata.occ_distance2 * 256);
                        detect_distance_sensor->publish_state(lddataUnion.lddata.detect_distance1 + lddataUnion.lddata.detect_distance2 * 256);
                        if (lddataUnion.lddata.data_mode == 1) //工程模式
                        {
                            en_mov_0->publish_state(lddataUnion.lddata.en_mov_0);
                            en_mov_1->publish_state(lddataUnion.lddata.en_mov_1);
                            en_mov_2->publish_state(lddataUnion.lddata.en_mov_2);
                            en_mov_3->publish_state(lddataUnion.lddata.en_mov_3);
                            en_mov_4->publish_state(lddataUnion.lddata.en_mov_4);
                            en_mov_5->publish_state(lddataUnion.lddata.en_mov_5);
                            en_mov_6->publish_state(lddataUnion.lddata.en_mov_6);
                            en_mov_7->publish_state(lddataUnion.lddata.en_mov_7);
                            en_mov_8->publish_state(lddataUnion.lddata.en_mov_8);
                            en_occ_0->publish_state(lddataUnion.lddata.en_occ_0);
                            en_occ_1->publish_state(lddataUnion.lddata.en_occ_1);
                            en_occ_2->publish_state(lddataUnion.lddata.en_occ_2);
                            en_occ_3->publish_state(lddataUnion.lddata.en_occ_3);
                            en_occ_4->publish_state(lddataUnion.lddata.en_occ_4);
                            en_occ_5->publish_state(lddataUnion.lddata.en_occ_5);
                            en_occ_6->publish_state(lddataUnion.lddata.en_occ_6);
                            en_occ_7->publish_state(lddataUnion.lddata.en_occ_7);
                            en_occ_8->publish_state(lddataUnion.lddata.en_occ_8);
                        }
                        bytes.clear();
                    }
                } 
                else if (presence_state)
                {  
                    id(target_state).publish_state("none");
                    mov_distance_sensor->publish_state(0);
                    occ_distance_sensor->publish_state(0);
                    detect_distance_sensor->publish_state(0);
                    en_mov_0->publish_state(0);
                    en_mov_1->publish_state(0);
                    en_mov_2->publish_state(0);
                    en_mov_3->publish_state(0);
                    en_mov_4->publish_state(0);
                    en_mov_5->publish_state(0);
                    en_mov_6->publish_state(0);
                    en_mov_7->publish_state(0);
                    en_mov_8->publish_state(0);
                    en_occ_0->publish_state(0);
                    en_occ_1->publish_state(0);
                    en_occ_2->publish_state(0);
                    en_occ_3->publish_state(0);
                    en_occ_4->publish_state(0);
                    en_occ_5->publish_state(0);
                    en_occ_6->publish_state(0);
                    en_occ_7->publish_state(0);
                    en_occ_8->publish_state(0);
                    presence_state = false;
                    bytes.clear();
                } 
                else
                {
                    bytes.clear();
                }
            }
            else if (HeaderMatched(bytes, header_parameter))
            {
                if (bytes.size() < 11 + sizeof(PARAMETER))  
                {
                    continue;
                }
				
                PARAMETERUnion parameterUnion;
                std::copy(bytes.begin() + 11, (bytes.begin() + 11 + sizeof(PARAMETER)), parameterUnion.bytes);
                // ESP_LOGI("custom", "Sending Parameters");
				id(max_distance).publish_state(parameterUnion.parameter.max_distance * 0.75 );            // 最大距离门  
				id(max_mov_distance).publish_state(parameterUnion.parameter.max_mov_distance * 0.75 );    // 配置最大运动距离门
				id(max_occ_distance).publish_state(parameterUnion.parameter.max_occ_distance * 0.75 );    // 配置最大静止距离门
				id(mov_sn_distance_0).publish_state(parameterUnion.parameter.mov_sn_distance_0);          // 距离门0运动灵敏度，下同
				id(mov_sn_distance_1).publish_state(parameterUnion.parameter.mov_sn_distance_1);
				id(mov_sn_distance_2).publish_state(parameterUnion.parameter.mov_sn_distance_2);
				id(mov_sn_distance_3).publish_state(parameterUnion.parameter.mov_sn_distance_3);
				id(mov_sn_distance_4).publish_state(parameterUnion.parameter.mov_sn_distance_4);
				id(mov_sn_distance_5).publish_state(parameterUnion.parameter.mov_sn_distance_5);
				id(mov_sn_distance_6).publish_state(parameterUnion.parameter.mov_sn_distance_6);
				id(mov_sn_distance_7).publish_state(parameterUnion.parameter.mov_sn_distance_7);
				id(mov_sn_distance_8).publish_state(parameterUnion.parameter.mov_sn_distance_8);
				id(occ_sn_distance_0).publish_state(parameterUnion.parameter.occ_sn_distance_0);           // 距离门0静止灵敏度，下同
				id(occ_sn_distance_1).publish_state(parameterUnion.parameter.occ_sn_distance_1);
				id(occ_sn_distance_2).publish_state(parameterUnion.parameter.occ_sn_distance_2);
				id(occ_sn_distance_3).publish_state(parameterUnion.parameter.occ_sn_distance_3);
				id(occ_sn_distance_4).publish_state(parameterUnion.parameter.occ_sn_distance_4);
				id(occ_sn_distance_5).publish_state(parameterUnion.parameter.occ_sn_distance_5);
				id(occ_sn_distance_6).publish_state(parameterUnion.parameter.occ_sn_distance_6);
				id(occ_sn_distance_7).publish_state(parameterUnion.parameter.occ_sn_distance_7);
				id(occ_sn_distance_8).publish_state(parameterUnion.parameter.occ_sn_distance_8);
				id(none_duration).publish_state(parameterUnion.parameter.none_duration);     
		
                bytes.clear();
            }
/*       调试反馈用
            else if (HeaderMatched(bytes, header_feedback))
            {
                if (bytes.size() < 6 + sizeof(FEEDBACK))
                {
                    continue;
                }

                FEEDBACKUnion feedbackUnion;
                std::copy(bytes.begin() + 6, (bytes.begin() + 6 + sizeof(FEEDBACK)), feedbackUnion.bytes);
                switch (feedbackUnion.feedback.command1)
                {
                    case 254: //0xFE-配置结束
                        if (!feedbackUnion.feedback.result) ESP_LOGD("custom", "Config successed.");
                        else ESP_LOGI("custom", "!!!Config Failed!!!");
                        break;
                    case 96: //0x60-最大距离门与无人持续时间
                        if (!feedbackUnion.feedback.result) ESP_LOGD("custom", "Max distance setup successed.");
                        else ESP_LOGI("custom", "!!!Max distance setup Failed!!!");
                        break;
                    case 100: //0x64-距离灵敏度配置
                        if (!feedbackUnion.feedback.result) ESP_LOGD("custom", "Sensitivity setup successed.");
                        else ESP_LOGI("custom", "!!!Sensitivity setup Failed!!!");
                        break;
                    case 162: //0xA2-恢复出厂设置  旧版本无用
                        if (!feedbackUnion.feedback.result) ESP_LOGD("custom", "Set default successed.");
                        else ESP_LOGI("custom", "!!!Set default Failed!!!");
                        break;
                }

                bytes.clear();
            }
*/
            else
            {
                bytes.erase(bytes.begin());
                continue;
            }
        }
    }

    typedef struct
    {
        uint8_t max_distance;            // 最大距离门
		uint8_t max_mov_distance;        // 配置最大运动距离门
		uint8_t max_occ_distance;        // 配置最大静止距离门
		uint8_t mov_sn_distance_0;       // 距离门0运动灵敏度，下同
		uint8_t mov_sn_distance_1; 
		uint8_t mov_sn_distance_2;
		uint8_t mov_sn_distance_3;
		uint8_t mov_sn_distance_4;
		uint8_t mov_sn_distance_5;
		uint8_t mov_sn_distance_6;
		uint8_t mov_sn_distance_7;
		uint8_t mov_sn_distance_8;
		uint8_t occ_sn_distance_0;       // 距离门0静止灵敏度，下同
		uint8_t occ_sn_distance_1;
		uint8_t occ_sn_distance_2;
		uint8_t occ_sn_distance_3;
		uint8_t occ_sn_distance_4;
		uint8_t occ_sn_distance_5;
		uint8_t occ_sn_distance_6;
		uint8_t occ_sn_distance_7;
		uint8_t occ_sn_distance_8;
		uint8_t none_duration;            // 无人持续时间
    } PARAMETER	; // 参数
	
    typedef union
    {
        PARAMETER parameter;
        uint8_t bytes[sizeof(PARAMETER)];
    } PARAMETERUnion;
	
    typedef struct
    {
        uint8_t data_mode;     //数据类型 01-工程模式，02-基本
        uint8_t data_header;     //头部  
        uint8_t target_state;     //目标状态  
        uint8_t mov_distance1;   //运动目标距离
        uint8_t mov_distance2;
        uint8_t mov_energy;      //运动目标能量
        uint8_t occ_distance1;   //静止目标距离
        uint8_t occ_distance2;
        uint8_t occ_energy;      //静止目标能量
        uint8_t detect_distance1;     //探测距离
        uint8_t detect_distance2;
        //能量数据 工程模式下有效
        uint8_t en_mov_0;
        uint8_t en_mov_1;
        uint8_t en_mov_2;
        uint8_t en_mov_3;
        uint8_t en_mov_4;
        uint8_t en_mov_5;
        uint8_t en_mov_6;
        uint8_t en_mov_7;
        uint8_t en_mov_8;
        uint8_t en_occ_0;
        uint8_t en_occ_1;
        uint8_t en_occ_2;
        uint8_t en_occ_3;
        uint8_t en_occ_4;
        uint8_t en_occ_5;
        uint8_t en_occ_6;
        uint8_t en_occ_7;
        uint8_t en_occ_8;
    } LDDATA;    //雷达探测数据

    typedef union
    {
        LDDATA lddata;
        uint8_t bytes[sizeof(LDDATA)];
    } LDDATAUnion;
/*    调试用
    typedef struct
    {
        uint8_t command1;     //指令
        uint8_t command2;     //指令  
        uint8_t result;       //反馈  
    } FEEDBACK;

    typedef union
    {
        FEEDBACK feedback;
        uint8_t bytes[sizeof(FEEDBACK)];
    } FEEDBACKUnion;
*/
};