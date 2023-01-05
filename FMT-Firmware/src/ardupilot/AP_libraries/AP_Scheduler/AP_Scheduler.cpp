/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  main loop scheduler for APM
 *  Author: Andrew Tridgell, January 2013
 *
 */
#include "AP_Scheduler.h"

#include "ap_hal.h"
#include "AP_Vehicle.h"
#include <stdio.h>

#define SCHEDULER_DEFAULT_LOOP_RATE 500

extern AP_HAL hal;

int8_t AP_Scheduler::current_task = -1;

// constructor
AP_Scheduler::AP_Scheduler(void):
    _debug(0),
    _loop_rate_hz(SCHEDULER_DEFAULT_LOOP_RATE)
{
    // only allow 50 to 400 Hz
    if (_loop_rate_hz < 50) {
        _loop_rate_hz=50;
    } else if (_loop_rate_hz > 500) {
        _loop_rate_hz=500;
    }
}

// initialise the scheduler
void AP_Scheduler::init(const AP_Scheduler::Task *tasks, uint8_t num_tasks)
{
    _tasks = tasks;
    _num_tasks = num_tasks;
    _last_run = new uint16_t[_num_tasks];
    memset(_last_run, 0, sizeof(_last_run[0]) * _num_tasks);
    _tick_counter = 0;
    printf(" AP_Scheduler number of task %d\n",_num_tasks);

    for (uint8_t i=0; i<_num_tasks; i++) {
        printf("  --[%d] name: %s()  max_micros: %d  rate_hz: %.0f\n",i, _tasks[i].name, _tasks[i].max_time_micros, _tasks[i].rate_hz);
    }
}

// one tick has passed
void AP_Scheduler::tick(void)
{
    _tick_counter++;
}

/*
  run one tick
  this will run as many scheduler tasks as we can in the specified time
 */
void AP_Scheduler::run(uint32_t time_available)
{
    uint32_t run_started_usec = hal.micros();
    uint32_t now = run_started_usec;

    // if (_debug > 1 && _perf_counters == nullptr) {
    //     _perf_counters = new AP_HAL::Util::perf_counter_t[_num_tasks];
    //     if (_perf_counters != nullptr) {
    //         for (uint8_t i=0; i<_num_tasks; i++) {
    //             // _perf_counters[i] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _tasks[i].name);
    //         }
    //     }
    // }
    
    for (uint8_t i=0; i<_num_tasks; i++) {
        uint16_t dt = _tick_counter - _last_run[i];
        uint16_t interval_ticks = (float)_loop_rate_hz / _tasks[i].rate_hz;
        if (interval_ticks < 1) {
            interval_ticks = 1;
        }
        if (dt >= interval_ticks) {
            // this task is due to run. Do we have enough time to run it?
            _task_time_allowed = _tasks[i].max_time_micros;

            if (dt >= interval_ticks*2) {
                // we've slipped a whole run of this task!
                if (_debug > 4) {
                    printf("Scheduler slip task[%u-%s] (%u/%u/%u)\n",
                             (unsigned)i,
                             _tasks[i].name,
                             (unsigned)dt,
                             (unsigned)interval_ticks,
                             (unsigned)_task_time_allowed);
                }
            }

            if (_task_time_allowed <= time_available) {
                // run it
                _task_time_started = now;
                current_task = i;
                // if (_debug > 1 && _perf_counters && _perf_counters[i]) {
                //     hal.util->perf_begin(_perf_counters[i]);
                // }
                _tasks[i].function();
                // if (_debug > 1 && _perf_counters && _perf_counters[i]) {
                //     hal.util->perf_end(_perf_counters[i]);
                // }
                current_task = -1;

                // record the tick counter when we ran. This drives
                // when we next run the event
                _last_run[i] = _tick_counter;

                // work out how long the event actually took
                now = hal.micros();
                uint32_t time_taken = now - _task_time_started;

                if (time_taken > _task_time_allowed) {
                    // the event overran!
                    // if (_debug > 4) {
                    //     ::printf("Scheduler overrun task[%u-%s] (%u/%u)\n",
                    //              (unsigned)i,
                    //              _tasks[i].name,
                    //              (unsigned)time_taken,
                    //              (unsigned)_task_time_allowed);
                    // }
                    ;
                }
                if (time_taken >= time_available) {
                    goto update_spare_ticks;
                }
                time_available -= time_taken;
            }
        }
    }

    // update number of spare microseconds
    _spare_micros += time_available;

update_spare_ticks:
    _spare_ticks++;
    if (_spare_ticks == 32) {
        _spare_ticks /= 2;
        _spare_micros /= 2;
    }
}

/*
  return number of micros until the current task reaches its deadline
 */
uint16_t AP_Scheduler::time_available_usec(void)
{
    uint32_t dt = hal.micros() - _task_time_started;
    if (dt > _task_time_allowed) {
        return 0;
    }
    return _task_time_allowed - dt;
}

/*
  calculate load average as a number from 0 to 1
 */
float AP_Scheduler::load_average(uint32_t tick_time_usec) const
{
    if (_spare_ticks == 0) {
        return 0.0f;
    }
    uint32_t used_time = tick_time_usec - (_spare_micros/_spare_ticks);
    return used_time / (float)tick_time_usec;
}
