#include "../inc/MarlinConfig.h"
#include "PeripheManager.h"
#include "../MarlinCore.h"
#include "../core/debug_out.h"

PeripheManager Periphe;

void PeripheManager::Init()
{
    door_check_auto_switch = false;
    last_door_status = false;
}

void PeripheManager::DoorCheck()
{
    if (door_check_auto_switch){
        //Door open
        if (READ(DOOR_STATUS_PIN)){
            SERIAL_ECHOLNPGM_P("warning:Door Openned");
        }else {
            //Door close
            SERIAL_ECHOLNPGM_P("warning:Door Closed");
        }
    }  
}

void PeripheManager::DoorCheckAuto()
{
    if (door_check_auto_switch){
        bool door_status = READ(DOOR_STATUS_PIN);
        if (door_status != last_door_status){
            last_door_status = door_status;
            SERIAL_ECHO("warning:Door ");
            SERIAL_ECHOLN(last_door_status? "Openned" : "Closed");
        }
    }
    
}

void PeripheManager::SetDoorCheckStatus(bool Enable)
{
    if (Enable){
        door_check_auto_switch = true;
        DEBUG_PRINT_P("Auto door check on!\n");
    }else {
        door_check_auto_switch = false;
        DEBUG_PRINT_P("Auto door check off!\n");
    }
}