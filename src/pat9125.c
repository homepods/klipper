// Support for PAT9125 i2c laser sensor
//
// Copyright (C) 2018  Eric Callahan <arksine.code@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/irq.h" // irq_disable
#include "basecmd.h" //oid_alloc
#include "command.h"  //sendf
#include "sched.h" //DECL_COMMAND
#include "i2ccmds.h" // i2cdev_oid_lookup
#include "stepper.h" // stepper_oid_lookup

#define REG_PID         0x00
#define REG_MOTION      0x02
#define REG_DELTA_X     0x03
#define REG_DELTA_XYH   0x12
#define REG_SHUTTER     0x14
#define REG_FRAME       0x17

#define MAX_STATE_LEN 5

enum { FLAG_INIT=1<<0, FLAG_OQ_EN=1<<1, FLAG_UPDATING=1<<2, FLAG_MOTION=1<<7 };
enum { STEPPER_BIAS=0x40000000 };

struct pat9125_i2c {
    struct timer time;
    struct i2cdev_s* i2c;
    struct stepper* e_stepper;
    uint32_t rest_ticks;
    uint8_t flags;
    uint8_t reg_data[MAX_STATE_LEN];
    uint8_t data_len;
};

static struct task_wake pat9125_wake;

uint8_t
pat9125_i2c_write_verify(struct pat9125_i2c *pat, uint8_t* write,
                         uint8_t retries)
{
    uint8_t recd;
    while (retries--) {
        i2cdev_write(pat->i2c, 2, write);
        i2cdev_read(pat->i2c, 1, write, 1, &recd);
        if (write[1] == recd)
            return 1;
    }
    return 0;
}

static uint_fast8_t
pat9125_event(struct timer *t)
{
    struct pat9125_i2c *pat = container_of(t, struct pat9125_i2c, time);
    sched_wake_task(&pat9125_wake);
    // Reschedule timer
    pat->time.waketime += pat->rest_ticks;
    return SF_RESCHEDULE;
}

void
command_config_pat9125(uint32_t *args)
{
    struct pat9125_i2c *pat = oid_alloc(args[0],
        command_config_pat9125, sizeof(*pat));
    pat->i2c = i2cdev_oid_lookup(args[1]);
    pat->time.func = pat9125_event;
    pat->data_len = 3;
    pat->flags = FLAG_INIT;
    if (args[2]) {
        pat->data_len = 5;
        pat->flags |= FLAG_OQ_EN;
    }
}
DECL_COMMAND(command_config_pat9125,
             "command_config_pat9125 oid=%c i2c_oid=%c oq_enable=%c");

void
command_pat9125_write_verify(uint32_t *args)
{
    uint8_t oid = args[0];
    uint8_t data_len = args[1];
    uint8_t success = 1;
    if (data_len & 0x01 || data_len < 2) {
        // data must arrive in 8-bit pairs
        success = 0;
    }
    else {
        struct pat9125_i2c *pat = oid_lookup(oid, command_config_pat9125);
        uint8_t *data = (void*)(size_t)args[2];
        uint8_t retries = args[3];
        while (data_len > 0) {
            success = pat9125_i2c_write_verify(pat, data, retries);
            if (!success) {
                break;
            }
            data_len -= 2;
            data += 2;
        }
    }
    sendf("pat9125_verify_response oid=%c success=%c", oid, success);
}
DECL_COMMAND(command_pat9125_write_verify,
             "command_pat9125_write_verify oid=%c sequence=%*s retries=%u");

void
command_pat9125_start_update(uint32_t *args)
{
    uint8_t oid = args[0];
    struct pat9125_i2c *pat = oid_lookup(oid, command_config_pat9125);
    sched_del_timer(&pat->time);
    pat->e_stepper = stepper_oid_lookup(args[1]);
    pat->time.waketime = args[2];
    pat->rest_ticks = args[3];
    pat->flags |= FLAG_UPDATING;
    sched_add_timer(&pat->time);
}
DECL_COMMAND(command_pat9125_start_update,
             "command_pat9125_start_update oid=%c step_oid=%c clock=%u "
             "rest_ticks=%u");

void
command_pat9125_stop_update(uint32_t *args)
{
    uint8_t oid = args[0];
    struct pat9125_i2c *pat = oid_lookup(oid, command_config_pat9125);
    sched_del_timer(&pat->time);
    pat->flags &= ~FLAG_UPDATING;
}
DECL_COMMAND(command_pat9125_stop_update,
             "command_pat9125_stop_update oid=%c");

static inline void
pat9125_update(struct pat9125_i2c *pat, uint8_t oid)
{
    uint32_t position = 0;
    uint8_t* data = pat->reg_data;
    uint8_t reg;

    reg = REG_MOTION;
    i2cdev_read(pat->i2c, 1, &reg, 1, data);

    // Get current position after reading out motion
    irqstatus_t flag = irq_save();
    position = stepper_get_position(pat->e_stepper);
    irq_restore(flag);

    if (unlikely(*data == 0xFF)) {
        // ACK error, report zero status and shut down i2c
        pat->flags = 0;
        sched_del_timer(&pat->time);
        goto send;
    }

    pat->flags &= ~FLAG_MOTION;
    if (*data & FLAG_MOTION) {
        // read xyh
        pat->flags |= FLAG_MOTION;
        reg = REG_DELTA_X;
        i2cdev_read(pat->i2c, 1, &reg, 2, data);
        reg = REG_DELTA_XYH;
        i2cdev_read(pat->i2c, 1, &reg, 1, data + 2);
    }

    if (pat->flags & FLAG_OQ_EN) {
        reg = REG_SHUTTER;
        i2cdev_read(pat->i2c, 1, &reg, 1, data + 3);
        reg = REG_FRAME;
        i2cdev_read(pat->i2c, 1, &reg, 1, data + 4);
    }

    send:
    sendf("pat9125_update_response oid=%c epos=%i data=%*s flags=%c",
          oid, position - STEPPER_BIAS, pat->data_len, data, pat->flags);
}

void
pat9125_task(void)
{
    if (!sched_check_wake(&pat9125_wake))
        return;
    uint8_t oid;
    struct pat9125_i2c *p;
    foreach_oid(oid, p, command_config_pat9125) {
        pat9125_update(p, oid);
    }
}
DECL_TASK(pat9125_task);
