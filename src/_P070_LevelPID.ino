//#######################################################################################################
//#################################### Plugin 070: Level Control ########################################
//#######################################################################################################

#include <PID_v1.h>

#define PLUGIN_070
#define PLUGIN_ID_070        70
#define PLUGIN_NAME_070       "Level Control PID"
#define PLUGIN_VALUENAME1_070 "Output"

boolean Plugin_070(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;
  static byte switchstate[TASKS_MAX];
  double Setpoint, Input, Output;

  //Specify the links and initial tuning parameters
  PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

  int WindowSize = 5000;
  unsigned long windowStartTime;
  windowStartTime = millis();

  //initialize the variables we're linked to
  Setpoint = Settings.TaskDevicePluginConfigFloat[event->TaskIndex][0];

  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);


  switch (function)
  {

    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_070;
        Device[deviceCount].Type = DEVICE_TYPE_SINGLE;
        Device[deviceCount].VType = SENSOR_TYPE_SWITCH;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = false;
        Device[deviceCount].ValueCount = 1;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = false;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_070);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_070));
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {
        char tmpString[128];

        string += F("<TR><TD>Check Task:<TD>");
        addTaskSelect(string, "plugin_070_task", Settings.TaskDevicePluginConfig[event->TaskIndex][0]);

        LoadTaskSettings(Settings.TaskDevicePluginConfig[event->TaskIndex][0]); // we need to load the values from another task for selection!
        string += F("<TR><TD>Check Value:<TD>");
        addTaskValueSelect(string, "plugin_070_value", Settings.TaskDevicePluginConfig[event->TaskIndex][1], Settings.TaskDevicePluginConfig[event->TaskIndex][0]);

      	addFormTextBox(string, F("Set Value"), F("plugin_070_setvalue"), String(Settings.TaskDevicePluginConfigFloat[event->TaskIndex][0]), 8);

      	addFormTextBox(string, F("Hysteresis"), F("plugin_070_hyst"), String(Settings.TaskDevicePluginConfigFloat[event->TaskIndex][1]), 8);

        LoadTaskSettings(event->TaskIndex); // we need to restore our original taskvalues!
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        Settings.TaskDevicePluginConfig[event->TaskIndex][0] = getFormItemInt(F("plugin_070_task"));
        Settings.TaskDevicePluginConfig[event->TaskIndex][1] = getFormItemInt(F("plugin_070_value"));
        Settings.TaskDevicePluginConfigFloat[event->TaskIndex][0] = getFormItemFloat(F("plugin_070_setvalue"));
        Settings.TaskDevicePluginConfigFloat[event->TaskIndex][1] = getFormItemFloat(F("plugin_070_hyst"));
        success = true;
        break;
      }

    case PLUGIN_REMOTE_CONFIG:
      {
        Serial.print("levelplugin: ");
        Serial.println(string);
        String command = parseString(string, 1);
        if (command == F("setlevel"))
        {
          String value = parseString(string, 2);
          Settings.TaskDevicePluginConfigFloat[event->TaskIndex][0] = value.toFloat();
          Serial.println(value);
          SaveSettings();
          success = true;
        }
        break;
      }

    case PLUGIN_INIT:
      {
        Serial.print(F("INIT : Output "));
        Serial.println(Settings.TaskDevicePin1[event->TaskIndex]);
        pinMode(Settings.TaskDevicePin1[event->TaskIndex], OUTPUT);
        success = true;
        break;
      }
      case PLUGIN_WRITE:
      {
         String command = parseString(string, 1);
         if (command == F("getlevel"))
         {
                 int task = getParamStartPos(string, 2);
                 String configTaskName = string.substring(task);
                 int8_t index = getTaskIndexByName(configTaskName);
                 if (index != -1)
                 {
                       event->TaskIndex = index;
                       byte TaskIndex = Settings.TaskDevicePluginConfig[event->TaskIndex][0];
                       byte BaseVarIndex = TaskIndex * VARS_PER_TASK + Settings.TaskDevicePluginConfig[event->TaskIndex][1];
                       float value = UserVar[BaseVarIndex];
                       Serial.print(configTaskName);
                       Serial.print(" ");
                       Serial.print(value);
                       Serial.print("\n");
                       success = true;
                 }
         }
         if (command == F("setlevel"))
 {
                 int configCommandPos1 = getParamStartPos(string, 2);
                 int configCommandPos2 = getParamStartPos(string, 3);

                 String configTaskName = string.substring(configCommandPos1, configCommandPos2 - 1);
                 String configTaskValue = string.substring(configCommandPos2);

                 int8_t index = getTaskIndexByName(configTaskName);
                 if (index != -1 && IsNumeric(configTaskValue))
                 {
                       event->TaskIndex = index;
                       byte TaskIndex = Settings.TaskDevicePluginConfig[event->TaskIndex][0];
                       byte BaseVarIndex = TaskIndex * VARS_PER_TASK + Settings.TaskDevicePluginConfig[event->TaskIndex][1];
                       Settings.TaskDevicePluginConfigFloat[event->TaskIndex][0] = configTaskValue.toFloat();
                       SaveSettings();
                       success = true;
                 }
         }

      }
      break;

    case PLUGIN_ONCE_A_SECOND:
      {
      // we're checking a var from another task, so calculate that basevar
        byte TaskIndex = Settings.TaskDevicePluginConfig[event->TaskIndex][0];
        byte BaseVarIndex = TaskIndex * VARS_PER_TASK + Settings.TaskDevicePluginConfig[event->TaskIndex][1];
        //Define Variables we'll be connecting to

        float value = UserVar[BaseVarIndex];
        Input = value;
        myPID.Compute();
          /************************************************
           * turn the output pin on/off based on pid output
           ************************************************/
          unsigned long now = millis();
          if(now - windowStartTime>WindowSize)
          { //time to shift the Relay Window
            windowStartTime += WindowSize;
          }
          if(Output > now - windowStartTime) {
            digitalWrite(Settings.TaskDevicePin1[event->TaskIndex],1);
            Serial.print(F("OutPID : State "));
            Serial.println("1");
          }
          else {
          digitalWrite(Settings.TaskDevicePin1[event->TaskIndex],0);
          Serial.print(F("OutPID : State "));
          Serial.println("0");
        }
        sendData(event);
        success = true;
        break;
      }

  }
  return success;
}
