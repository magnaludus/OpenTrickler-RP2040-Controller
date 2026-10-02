"""Host regression tests for actual firmware routines with hardware/RTOS stubs.
Run: python3 tests/control_regressions.py (requires a host C compiler).
"""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
# CC overrides the host compiler, e.g.  CC="python -m ziglang cc" python tests/control_regressions.py
COMPILER = shlex.split(os.environ.get("CC", "cc"))


def function(path, name):
    source = (ROOT / path).read_text()
    match = re.search(r"^.*\b" + name + r"\([^;]*?\)\s*\{", source, re.M)
    assert match, name
    start = match.start()
    opening = source.index("{", match.start())
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def run(name, source, defines=(), arguments=()):
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "test.c"
        path.write_text(source)
        binary = Path(folder) / "test"
        subprocess.run([*COMPILER, "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=undefined",
                        "-fno-sanitize-recover=all", *defines, str(path), "-lm", "-o", str(binary)], check=True)
        subprocess.run([str(binary), *arguments], check=True)
    print(name + ": passed")


common = """
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
"""

motors = (ROOT / "src/motors.c").read_text()
queue_creation = "\n".join(re.findall(r"^    (?:coarse|fine)_trickler_motor_config.stepper_speed_control_queue = xQueueCreate.*;", motors, re.M))
assert len(queue_creation.splitlines()) == 2
motor_test = common + r"""
#define STEPPER_LOW_CYCLE_COUNT 13
#define MAX_RESPONSE_TIME .01f
#define portMAX_DELAY 0
#define SELECT_COARSE_TRICKLER_MOTOR 0
#define SELECT_FINE_TRICKLER_MOTOR 1
#define SELECT_BOTH_MOTOR 2
typedef int motor_select_t;
typedef struct {size_t size; unsigned char bytes[16];} Queue;
typedef struct {
    struct {float angular_acceleration; uint32_t full_steps_per_rotation, microsteps;} persistent_config;
    struct {int pio,sm;} pio_config;
    Queue *stepper_speed_control_queue;
} motor_config_t;
static motor_config_t coarse_trickler_motor_config, fine_trickler_motor_config;
static Queue queues[2];
static int queue_count;
static Queue *xQueueCreate(int length, size_t size) {
    assert(length==2); queues[queue_count].size=size; return &queues[queue_count++];
}
static void xQueueSend(Queue *q, const void *value, int timeout) {
    (void)timeout; memcpy(q->bytes,value,q->size);
}
static uint32_t clock_us, clock_calls, final_period;
static uint32_t time_us_32(void) {assert(++clock_calls<10000); uint32_t now=clock_us; clock_us+=1000; return now;}
static void pio_sm_clear_fifos(int pio,int sm) {(void)pio;(void)sm;}
static void pio_sm_put(int pio,int sm,uint32_t period) {(void)pio;(void)sm;(void)period;}
static void pio_sm_put_blocking(int pio,int sm,uint32_t period) {(void)pio;(void)sm;final_period=period;}
""" + function("src/motors.c", "speed_to_period") + function("src/motors.c", "speed_ramp") + function("src/motors.c", "motor_set_speed") + "\nint main(void) {\n" + queue_creation + r"""
    assert(queues[0].size==sizeof(float) && queues[1].size==sizeof(float));
    motor_set_speed(SELECT_BOTH_MOTOR,1.25f);
    struct {float value; uint32_t guard;} received={0,0x12345678};
    memcpy(&received.value,queues[0].bytes,queues[0].size);
    assert(received.value==1.25f && received.guard==0x12345678);
    motor_set_speed(SELECT_BOTH_MOTOR,NAN);
    memcpy(&received.value,queues[1].bytes,queues[1].size); assert(received.value==0);
    assert(speed_to_period(0,125000000,51200)==0);
    assert(speed_to_period(NAN,125000000,51200)==0);
    assert(speed_to_period(INFINITY,125000000,51200)==0);
    assert(speed_to_period(-1,125000000,51200)==0);
    assert(speed_to_period(1e-30f,125000000,51200)==0);
    assert(speed_to_period(1,125000000,0)==0);
    assert(speed_to_period(1,125000000,51200)==2428);
    motor_config_t motor={.persistent_config={50,200,256}};
    clock_calls=0; clock_us=0; speed_ramp(&motor,0,0,125000000);
    assert(final_period==0 && clock_calls==2);
    clock_calls=0; clock_us=UINT32_MAX-5000; speed_ramp(&motor,0,1,125000000);
    assert(clock_calls>=20 && final_period==2428);
    clock_calls=0; clock_us=UINT32_MAX-5000; speed_ramp(&motor,1,0,125000000);
    assert(clock_calls>=20 && final_period==0);
    motor.persistent_config.angular_acceleration=0;
    clock_calls=0; speed_ramp(&motor,0,1,125000000); assert(final_period==2428);
    return 0;
}
"""
run("Motor queue, stop commands, unchanged speed, ramp rollover", motor_test)

header = (ROOT / "src/ota.h").read_text()
ota_defines = "\n".join(line for line in header.splitlines() if line.startswith("#define OTA_"))
ota_enum = re.search(r"typedef enum \{.*?\} ota_state_t;", header, re.S).group()
ota_test = common + ota_defines + "\n" + ota_enum + r"""
typedef int err_t;
typedef uint16_t u16_t;
typedef uint8_t u8_t;
#define ERR_VAL -1
#define ERR_OK 0
struct pbuf {void *payload; uint16_t len,tot_len; struct pbuf *next;};
static ota_state_t ota_state=OTA_STATE_IDLE;
static uint32_t ota_expected,ota_received,ota_crc,sector_fill,sector_index;
static uint8_t sector_buf[OTA_SECTOR_BYTES];
static void *ota_connection;
static bool write_ok=true;
static unsigned writes,frees;
static void set_msg(const char *s) {(void)s;}
static void pbuf_free(struct pbuf *p) {(void)p; frees++;}
static bool flash_write_sector(uint32_t offset,const uint8_t *data,uint32_t len) {
    (void)data; assert(offset>=OTA_STAGE_OFFSET); assert(offset+len<=PICO_FLASH_SIZE_BYTES); writes++; return write_ok;
}
""" + function("src/ota.c", "httpd_post_begin") + function("src/ota.c", "flush_sector") + function("src/ota.c", "httpd_post_receive_data") + function("src/ota.c", "httpd_post_finished") + r"""
int main(void) {
    assert(OTA_STAGE_OFFSET+OTA_MAX_IMAGE_BYTES<=PICO_FLASH_SIZE_BYTES);
    assert(OTA_MAX_IMAGE_BYTES<=OTA_STAGE_OFFSET);
    int one,two;
    char response[80]; u8_t window;
    assert(httpd_post_begin(&one,"/ota/upload","",0,10,response,sizeof(response),&window)==ERR_OK);
    assert(httpd_post_begin(&two,"/ota/upload","",0,10,response,sizeof(response),&window)==ERR_VAL);
    assert(ota_connection==&one && ota_expected==10 && ota_state==OTA_STATE_RECEIVING);
    httpd_post_finished(&two,response,sizeof(response)); assert(ota_connection==&one);
    char payload[20]={0}; struct pbuf p={payload,11,11,NULL};
    assert(httpd_post_receive_data(&one,&p)==ERR_VAL && writes==0 && frees==1);
    ota_state=OTA_STATE_IDLE;
    assert(httpd_post_begin(&one,"/ota/upload","",0,10,response,sizeof(response),&window)==ERR_OK);
    p.len=p.tot_len=10;
    assert(httpd_post_receive_data(&one,&p)==ERR_OK);
    httpd_post_finished(&one,response,sizeof(response)); assert(ota_state==OTA_STATE_RECEIVED && writes==1);
    ota_state=OTA_STATE_APPLYING;
    assert(httpd_post_begin(&two,"/ota/upload","",0,-1,response,sizeof(response),&window)==ERR_VAL);
    assert(ota_state==OTA_STATE_APPLYING);
    ota_state=OTA_STATE_IDLE;
    assert(httpd_post_begin(&one,"/ota/upload","",0,OTA_MAX_IMAGE_BYTES+1,response,sizeof(response),&window)==ERR_VAL);
    ota_state=OTA_STATE_IDLE;
    assert(httpd_post_begin(&one,"/ota/upload","",0,10,response,sizeof(response),&window)==ERR_OK);
    p.len=p.tot_len=5; assert(httpd_post_receive_data(&one,&p)==ERR_OK);
    httpd_post_finished(&one,response,sizeof(response)); assert(ota_state==OTA_STATE_ERROR);
    ota_state=OTA_STATE_IDLE; write_ok=false;
    assert(httpd_post_begin(&one,"/ota/upload","",0,10,response,sizeof(response),&window)==ERR_OK);
    p.len=p.tot_len=10; assert(httpd_post_receive_data(&one,&p)==ERR_OK);
    httpd_post_finished(&one,response,sizeof(response)); assert(ota_state==OTA_STATE_ERROR);
    return 0;
}
"""
for mb in (2,4):
    run(f"OTA bounds and upload lifecycle ({mb} MB flash)", ota_test, [f"-DPICO_FLASH_SIZE_BYTES={mb*1024*1024}"])

charge_header = (ROOT / 'src/charge_mode.h').read_text()
charge_types = '\n'.join(re.findall(r'typedef struct \{.*?\} \w+;', charge_header, re.S))
profile_type = re.search(r'typedef struct\s*\{.*?\} profile_t;', (ROOT / 'src/profile.h').read_text(), re.S).group()
charge_test = common + r'''
typedef uint32_t TickType_t;
typedef int decimal_places_t;
typedef uint32_t rgbw_u32_t;
typedef int motor_select_t;
#define DP_2 2
#define SELECT_COARSE_TRICKLER_MOTOR 0
#define SELECT_FINE_TRICKLER_MOTOR 1
#define SELECT_BOTH_MOTOR 2
#define BUTTON_RST_PRESSED 1
#define CHARGE_MODE_EXIT 0
#define CHARGE_MODE_WAIT_FOR_COMPLETE 2
#define CHARGE_MODE_WAIT_FOR_CUP_REMOVAL 3
#define CHARGE_SCALE_TIMEOUT_MS 2000u
#define SERVO_GATE_RATIO_OPEN 0
#define SERVO_GATE_RATIO_CLOSED 1
#define WEIGHT_STRING_LEN 8
#define portTICK_PERIOD_MS 1
#define portTICK_RATE_MS 1
#define pdMS_TO_TICKS(n) (n)
typedef int charge_mode_state_t;
typedef int ButtonEncoderEvent_t;
#define PROFILE_NAME_MAX_LEN 16
''' + charge_types + profile_type + r'''
static charge_mode_config_t charge_mode_config;
static profile_t profile;
static struct {struct {bool servo_gate_enable;} eeprom_servo_gate_config;} servo_gate;
static TickType_t now_tick,charge_start_tick,coarse_backoff_end_tick;
static float last_charge_elapsed_seconds,last_coarse_elapsed_seconds,last_coarse_stop_weight,weight_at_stop,rate_at_stop,last_dead_time_s;
static bool coarse_backoff_in_progress,throw_pending_record;
static char title_string[30];
static int scenario,sample_count,motor_calls;
static float speeds[2],first_fine_speed;
static TickType_t xTaskGetTickCount(void) {return now_tick;}
static int button_wait_for_input(bool block) {(void)block; return scenario==3?BUTTON_RST_PRESSED:0;}
static bool scale_block_wait_for_next_measurement(uint32_t timeout,float *weight) {
    if(scenario==1) {now_tick+=timeout;return false;}
    if(scenario==2) {now_tick+=timeout;*weight=NAN;return true;}
    *weight=sample_count++?9.99f:0.0f; return true;
}
static void motor_set_speed(int motor,float speed) {
    assert(isfinite(speed));motor_calls++;
    if(motor==SELECT_BOTH_MOTOR) speeds[0]=speeds[1]=speed;
    else {speeds[motor]=speed;if(motor==SELECT_FINE_TRICKLER_MOTOR && first_fine_speed<0) first_fine_speed=speed;}
}
static float get_motor_min_speed(int motor) {(void)motor;return .05f;}
static float get_motor_max_speed(int motor) {(void)motor;return 5.0f;}
static profile_t *profile_get_selected(void) {return &profile;}
static float charge_mode_get_active_bracket(void) {return .02f;}
static uint32_t session_backlight(void) {return 0;}
static void neopixel_led_set_colour(uint32_t a,uint32_t b,uint32_t c,bool d) {(void)a;(void)b;(void)c;(void)d;}
static void servo_gate_set_ratio(float ratio,bool wait) {(void)ratio;(void)wait;}
static void float_to_string(char *buffer,float weight,decimal_places_t places) {(void)places;snprintf(buffer,WEIGHT_STRING_LEN,"%.2f",weight);}
static void coarse_trickler_backoff(float speed) {(void)speed;}
static float handoff_for_target(float target) {
    float handoff=charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold, cap=.5f*target;
    if(cap>0 && handoff>cap) handoff=cap;
    return handoff<0?0:handoff;
}
static float learn_landing_trim(void) {return 0.0f;}
static void vTaskDelay(uint32_t ticks) {now_tick+=ticks;}
''' + function('src/charge_mode.cpp','charge_mode_wait_for_complete') + function('src/charge_mode.cpp','charge_mode_top_up') + r'''
static void setup(int mode) {
    memset(&charge_mode_config,0,sizeof(charge_mode_config));
    memset(&profile,0,sizeof(profile));
    charge_mode_config.target_charge_weight=10;
    charge_mode_config.eeprom_charge_mode_data.coarse_stop_threshold=1;
    charge_mode_config.charge_mode_state=CHARGE_MODE_WAIT_FOR_COMPLETE;
    profile.fine_kp=1;profile.coarse_kp=1;
    profile.fine_max_flow_speed_rps=2;profile.coarse_max_flow_speed_rps=2;
    profile.fine_min_flow_speed_rps=.05f;profile.coarse_min_flow_speed_rps=.05f;
    scenario=mode;now_tick=0;sample_count=0;motor_calls=0;
    speeds[0]=speeds[1]=1;first_fine_speed=-1;
}
int main(void) {
    setup(0);charge_mode_wait_for_complete();
    assert(first_fine_speed==2.0f); // zero-tick first sample must still use proportional control
    assert(speeds[0]==0 && speeds[1]==0 && charge_mode_config.charge_mode_state==CHARGE_MODE_WAIT_FOR_CUP_REMOVAL);
    setup(1);charge_mode_wait_for_complete();
    assert(now_tick==2000 && speeds[0]==0 && speeds[1]==0 && charge_mode_config.charge_mode_state==CHARGE_MODE_EXIT);
    setup(2);charge_mode_wait_for_complete();
    assert(now_tick==2000 && speeds[0]==0 && speeds[1]==0 && charge_mode_config.charge_mode_state==CHARGE_MODE_EXIT);
    setup(3);charge_mode_wait_for_complete();assert(speeds[0]==0 && speeds[1]==0);
    setup(1);assert(!charge_mode_top_up(.02f));assert(now_tick==2000 && speeds[1]==0);
    setup(2);assert(!charge_mode_top_up(.02f));assert(now_tick==2000 && speeds[1]==0);
    return 0;
}
'''
run('Charge PID first sample, scale loss, nonfinite readings, cancel and top-up', charge_test)

# Test the portal's actual UF2 parser against generated firmware and malformed files.
portal = (ROOT / 'src/html/web_portal.html').read_text()
constants = re.search(r'    const UF2_MAGIC0.*?    const FLASH_BASE[^\n]*;', portal, re.S).group()
parser = function('src/html/web_portal.html','uf2ToImage')
js_test = '''const fs = require('fs'); const assert = require('assert');\n''' + constants + '\n' + parser + r'''
function buffer(file) { const b=fs.readFileSync(file);return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength); }
function rejected(fn) {assert.throws(fn);}
for(const [board,family,capacity] of [['pico_w',RP2040_FAMILY,1048576],['pico2_w',RP2350_ARM_S,1572864]]) {
    const input=buffer(process.argv[2]+'/build_'+board+'/app.uf2');
    const image=uf2ToImage(input,family,capacity);
    const binary=fs.readFileSync(process.argv[2]+'/build_'+board+'/app.bin');
    assert(image.length>=binary.length && Buffer.from(image.slice(0,binary.length)).equals(binary));
    rejected(()=>uf2ToImage(input,family==RP2040_FAMILY?RP2350_ARM_S:RP2040_FAMILY,capacity));
    rejected(()=>uf2ToImage(input,family,4096));
    const damaged=input.slice(0);const view=new DataView(damaged);
    let flashBlock=0;while(view.getUint32(flashBlock+28,true)!=family) flashBlock+=512;
    view.setUint32(flashBlock+16,477,true);
    rejected(()=>uf2ToImage(damaged,family,capacity));
}
rejected(()=>uf2ToImage(new ArrayBuffer(0),RP2040_FAMILY,1048576));
rejected(()=>uf2ToImage(new ArrayBuffer(7),RP2040_FAMILY,1048576));
console.log('Portal UF2 parser: both real images, wrong board, oversize and malformed payloads passed');
'''
with tempfile.TemporaryDirectory() as folder:
    path=Path(folder)/'test.js';path.write_text(js_test)
    subprocess.run(['node',str(path),str(ROOT)],check=True)

vector_test = common + ota_defines + r'''
static uint32_t image[OTA_MAX_IMAGE_BYTES/4];
#define XIP_BASE ((uintptr_t)image-OTA_STAGE_OFFSET)
#define SRAM_BASE 0x20000000u
#if PICO_RP2350
#define SRAM_END 0x20082000u
#define VECTOR_WORD 0
#else
#define SRAM_END 0x20042000u
#define VECTOR_WORD 64
#endif
''' + function('src/ota.c','staged_image_plausible') + r'''
int main(int argc,char **argv) {
    assert(argc==2);FILE *file=fopen(argv[1],"rb");assert(file);
    uint32_t size=fread(image,1,sizeof(image),file);assert(!ferror(file));fclose(file);
    assert(staged_image_plausible(size));
    uint32_t original=image[VECTOR_WORD];
    image[VECTOR_WORD]=SRAM_END+8;assert(!staged_image_plausible(size));
    image[VECTOR_WORD]=original+1;assert(!staged_image_plausible(size));
    image[VECTOR_WORD]=original;
    image[VECTOR_WORD+1]=0x10000000u+size+1;assert(!staged_image_plausible(size));
    assert(!staged_image_plausible(100));
    return 0;
}
'''
for board,rp2350,mb in [('pico_w',0,2),('pico2_w',1,4)]:
    run(f'OTA vector validation ({board})',vector_test,
        [f'-DPICO_RP2350={rp2350}',f'-DPICO_FLASH_SIZE_BYTES={mb*1024*1024}'],
        [str(ROOT/f'build_{board}'/'app.bin')])

servo_test = common + r'''
#include <setjmp.h>
typedef float gate_ratio_t;
#define SERVO_GATE_RATIO_DISABLED -1.0f
#define GATE_DISABLED 0
#define GATE_OPEN 1
#define GATE_CLOSE 2
#define portMAX_DELAY 0
static struct {
    struct {float shutter_open_speed_pct_s,shutter_close_speed_pct_s;} eeprom_servo_gate_config;
    int control_queue,move_ready_semphore,gate_state;
    gate_ratio_t gate_ratio;
} servo_gate;
static jmp_buf done;
static unsigned samples,writes,signals,clock_calls;
static float last_ratio;
static uint32_t now_us=UINT32_MAX-5000;
static void xQueueReceive(int queue,gate_ratio_t *ratio,int timeout) {
    (void)queue;(void)timeout;
    if(samples==3) longjmp(done,1);
    const float requests[]={0,1,-1};*ratio=requests[samples++];
}
static void xSemaphoreGive(int semaphore) {(void)semaphore;signals++;}
static uint32_t time_us_32(void) {assert(++clock_calls<1000);uint32_t now=now_us;now_us+=1000;return now;}
static void _servo_gate_set_current_state(float ratio) {assert(isfinite(ratio) && ratio>=0 && ratio<=1);last_ratio=ratio;writes++;}
''' + function('src/servo_gate.c','clamp01') + function('src/servo_gate.c','servo_gate_control_task') + r'''
int main(void) {
    servo_gate.eeprom_servo_gate_config.shutter_open_speed_pct_s=5;
    servo_gate.eeprom_servo_gate_config.shutter_close_speed_pct_s=3;
    if(!setjmp(done)) servo_gate_control_task(NULL);
    assert(samples==3 && signals==3 && writes>300 && last_ratio==1);
    assert(servo_gate.gate_state==GATE_DISABLED && servo_gate.gate_ratio==1);
    return 0;
}
'''
run('Servo ramp across timer rollover and disable request',servo_test)

# Exercise the production motor task, not only the conversion/ramp helpers.
worker_test = motor_test.replace('#include <assert.h>', '#include <assert.h>\n#include <setjmp.h>')
worker_test = worker_test.replace('uint32_t full_steps_per_rotation, microsteps;', 'uint32_t full_steps_per_rotation, microsteps; float gear_ratio;')
worker_test = worker_test.replace('Queue *stepper_speed_control_queue;', 'Queue *stepper_speed_control_queue; float prev_velocity; bool step_direction; int dir_pin;')
preamble,main = worker_test.split('int main(void)',1)
preamble += r'''
#define clk_sys 0
static jmp_buf worker_done;
static unsigned reads;
static void xQueueReceive(Queue *q,void *value,int timeout) {
    (void)timeout;if(reads++) longjmp(worker_done,1);memcpy(value,q->bytes,q->size);
}
static uint32_t clock_get_hz(int clock) {(void)clock;return 125000000;}
static void gpio_put(int pin,bool state) {(void)pin;(void)state;}
''' + function('src/motors.c','stepper_speed_control_task')
main = main.replace('motor_config_t motor={.persistent_config={50,200,256}};', 'motor_config_t motor={.persistent_config={50,200,256,1.25f}};')
main = main.replace('    return 0;\n}', r'''
    coarse_trickler_motor_config.persistent_config=motor.persistent_config;
    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR,1.25f);
    reads=0;clock_calls=0;
    if(!setjmp(worker_done)) stepper_speed_control_task(&coarse_trickler_motor_config);
    assert(final_period==2428 && coarse_trickler_motor_config.prev_velocity==1);
    motor_set_speed(SELECT_COARSE_TRICKLER_MOTOR,0);
    reads=0;clock_calls=0;
    if(!setjmp(worker_done)) stepper_speed_control_task(&coarse_trickler_motor_config);
    assert(final_period==0 && coarse_trickler_motor_config.prev_velocity==0);
    return 0;
}''')
run('Motor worker receives coarse probe and stop commands',preamble+'int main(void)'+main)

pio_test = common + r'''
typedef unsigned uint;
typedef int PIO;
#define MOTOR_PIO 1
typedef struct {uint length;} pio_program_t;
static pio_program_t stepper_program={7};
typedef struct {uint step_pin;struct {PIO pio;uint sm;} pio_config;} motor_config_t;
static PIO stepper_program_pio=0;
static uint stepper_program_offset;
static bool stepper_program_loaded;
static bool room=true,claim_fails;
static uint claims,loads,inits,enables,zeros;
static uint pins[4],sms[4],offsets[4];
static int pio_claim_unused_sm(PIO pio,bool required) {(void)pio;(void)required;return claims++;}
static bool pio_can_add_program(PIO pio,const pio_program_t *program) {(void)pio;(void)program;return room;}
static uint pio_add_program(PIO pio,const pio_program_t *program) {(void)pio;(void)program;loads++;return 5;}
static void pio_sm_unclaim(PIO pio,uint sm) {(void)pio;(void)sm;}
static bool pio_claim_free_sm_and_add_program_for_gpio_range(const pio_program_t *program,PIO *pio,uint *sm,uint *offset,uint pin,uint count,bool set_base) {
    (void)program;(void)pin;assert(count==1 && set_base);
    if(claim_fails) return false;
    *pio=2;*sm=1;*offset=9;return true;
}
static void record_pio_failure(void) {}
static char pio_err_detail[24];
static void stepper_program_init(PIO pio,uint sm,uint offset,uint pin) {
    assert(pio==1 || pio==2);pins[inits]=pin;sms[inits]=sm;offsets[inits]=offset;inits++;
}
static void pio_sm_put_blocking(PIO pio,uint sm,uint period) {(void)pio;(void)sm;assert(period==0);zeros++;}
static void pio_sm_set_enabled(PIO pio,uint sm,bool enabled) {(void)pio;(void)sm;assert(enabled && inits>enables && zeros>enables);enables++;}
''' + function('src/motors.c','driver_pio_init') + r'''
int main(void) {
    motor_config_t coarse={.step_pin=10},fine={.step_pin=13};
    assert(driver_pio_init(&coarse) && driver_pio_init(&fine));
    assert(loads==1 && inits==2 && enables==2 && zeros==2);
    assert(pins[0]==10 && pins[1]==13 && sms[0]!=sms[1] && offsets[0]==5 && offsets[1]==5);
    stepper_program_loaded=false;room=false;
    assert(driver_pio_init(&coarse));assert(coarse.pio_config.pio==2 && offsets[2]==9);
    assert(inits==3 && enables==3 && zeros==3);
    claim_fails=true;assert(!driver_pio_init(&fine));assert(inits==3 && enables==3);
    return 0;
}
'''
run('PIO motor initialization: shared program, distinct SMs, fallback and failure',pio_test)
