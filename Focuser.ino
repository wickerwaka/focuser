#include <AccelStepper.h>

#include "SaveState.h"

/***************************
 Pin Definitions
 ***************************/
#define STP 16
#define DIR 10
#define MS1 14
#define MS2 18
#define EN 15

/***************************
 We use 1/8 stepping (MS1/MS2 both high)
 However move resolution is a full step
 ***************************/
#define MICROSTEPS 8
#define MICROSTEP_SHIFT 3

AccelStepper stepper(AccelStepper::DRIVER, STP, DIR);

enum ValueType {
  TY_NONE,
  TY_SIGNED_8,
  TY_UNSIGNED_8,
  TY_SIGNED_16,
  TY_UNSIGNED_16
};

typedef unsigned short (*CommandHandler)(unsigned short);
struct Command
{
  const char* mnemonic;
  ValueType input_type;
  ValueType output_type;
  CommandHandler handler;
};

long g_target_position = 0;
unsigned short g_speed = 2;
bool g_motor_enabled = false;

void enable_motor(bool en) {
  if (en == g_motor_enabled) {
    return;
  }
  
  if (en) {
    digitalWrite(EN, LOW);  
  } else {
    digitalWrite(EN, HIGH);
  }
  
  g_motor_enabled = en;
}

/***************************
 Command Handlers
 ***************************/

unsigned short on_get_position(unsigned short) {
  return (unsigned short)(stepper.currentPosition() >> MICROSTEP_SHIFT);
}

unsigned short on_get_target(unsigned short) {
  return (unsigned short)g_target_position;
}

unsigned short on_get_temperature(unsigned short) {
  return 32;
}

unsigned short on_get_speed(unsigned short) {
  return g_speed;
}

unsigned short on_get_step_mode(unsigned short) {
  return 0;
}

unsigned short on_is_running(unsigned short) {
  if (stepper.isRunning()) {
    return 1;
  }
  return 0;
}

unsigned short on_sync(unsigned short pos) {
  stepper.setCurrentPosition(pos << MICROSTEP_SHIFT);
  return 0;
}

unsigned short on_set_target(unsigned short pos) {
  g_target_position = pos;
  return 0;
}

unsigned short on_set_speed(unsigned short speed) {
  g_speed = speed;
}

unsigned short on_focus_go(unsigned short) {
  stepper.moveTo(g_target_position << MICROSTEP_SHIFT);
  return 0;
}

unsigned short on_focus_abort(unsigned short) {
  stepper.moveTo(stepper.currentPosition() & (MICROSTEPS - 1));
  return 0;
}

const Command g_commands[] = {
  { "PO", TY_SIGNED_8,    TY_NONE,        NULL },
  { "PS", TY_SIGNED_8,    TY_NONE,        NULL },
  { "PR", TY_UNSIGNED_8,  TY_NONE,        NULL },
  { "PG", TY_UNSIGNED_8,  TY_NONE,        NULL },
  { "PB", TY_UNSIGNED_8,  TY_NONE,        NULL },
  { "PC", TY_UNSIGNED_8,  TY_NONE,        NULL },
  { "PX", TY_UNSIGNED_16, TY_NONE,        NULL },
  { "PY", TY_UNSIGNED_16, TY_NONE,        NULL },
  { "PH", TY_UNSIGNED_8,  TY_NONE,        NULL },
  { "GP", TY_NONE,        TY_UNSIGNED_16, on_get_position },
  { "GN", TY_NONE,        TY_UNSIGNED_16, on_get_target },
  { "GT", TY_NONE,        TY_SIGNED_16,   on_get_temperature },
  { "GD", TY_NONE,        TY_UNSIGNED_8,  on_get_speed },
  { "GH", TY_NONE,        TY_UNSIGNED_8,  on_get_step_mode },
  { "GI", TY_NONE,        TY_UNSIGNED_8,  on_is_running },
  { "GB", TY_NONE,        TY_UNSIGNED_8,  NULL },
  { "GV", TY_NONE,        TY_UNSIGNED_8,  NULL },
  { "SP", TY_UNSIGNED_16, TY_NONE,        on_sync },
  { "SN", TY_UNSIGNED_16, TY_NONE,        on_set_target },
  { "SF", TY_NONE,        TY_NONE,        NULL },
  { "SH", TY_NONE,        TY_NONE,        NULL },
  { "SD", TY_UNSIGNED_8,  TY_NONE,        on_set_speed },
  { "FG", TY_NONE,        TY_NONE,        on_focus_go },
  { "FQ", TY_NONE,        TY_NONE,        on_focus_abort },
  { NULL, TY_NONE,        TY_NONE,        NULL }
};

/***************************
 Command Processing
 ***************************/

void process_command(const char* msg, int msg_size) {
  if (msg_size < 2) {
    return;
  }

  const Command* cmd = NULL;
  for (const Command* search = g_commands; search->mnemonic != NULL; search++) {
    if (search->mnemonic[0] == msg[0] && search->mnemonic[1] == msg[1]) {
      cmd = search;
      break;
    }
  }

  if (cmd == NULL) {
    return;
  }

  unsigned short input_value = (unsigned short)strtoul(msg + 2, 0, 16);

  switch (cmd->input_type) {
    case TY_NONE: input_value = 0; break;
    case TY_UNSIGNED_8:
      input_value &= 0xff;
      break;
    case TY_SIGNED_8:
      input_value = (unsigned short)((((short)input_value) << 8) >> 8);
      break;
    default:
      break;
  }

  unsigned short output_value = 0;
  if (cmd->handler != NULL) {
    output_value = cmd->handler(input_value);
  }

  char response[6];
  switch (cmd->output_type) {
    case TY_NONE: break;
    case TY_UNSIGNED_8:
    case TY_SIGNED_8:
      sprintf(response, "%02X#", output_value);
      Serial.print(response);
      break;
    case TY_UNSIGNED_16:
    case TY_SIGNED_16:
      sprintf(response, "%04X#", output_value);
      Serial.print(response);
      break;
  }
}

bool g_in_cmd = false;
char g_cmd_buf[8];
int g_cmd_buf_idx = 0;

void poll_serial() {
  while(Serial.available()) {
    char incoming = Serial.read();
    if (incoming == ':') {
      g_in_cmd = true;
      g_cmd_buf_idx = 0;
    } else if (incoming == '#') {
      g_cmd_buf[g_cmd_buf_idx] = '\0';
      process_command(g_cmd_buf, g_cmd_buf_idx);
      g_in_cmd = false;
    } else if (g_in_cmd) {
      if (g_cmd_buf_idx < (sizeof(g_cmd_buf) - 1)) {
        g_cmd_buf[g_cmd_buf_idx] = incoming;
        g_cmd_buf_idx++;
      }
    }
  }
}


/***************************
 Arduino Entry Points
 ***************************/

void setup() {
  Serial.begin(9600);
  pinMode(STP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(EN, OUTPUT);

  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);

  enable_motor(false);

  stepper.setAcceleration(200);
  stepper.setMaxSpeed(400);
  stepper.setMinPulseWidth(100);
}

void loop() {
  poll_serial();

  if (g_motor_enabled == false && (stepper.distanceToGo() != 0)) {
    enable_motor(true);
    delay(1);  
  }
  
  if (!stepper.run()) {
    if (g_motor_enabled) {
      delay(1);
      enable_motor(false);
    }
  }
}

