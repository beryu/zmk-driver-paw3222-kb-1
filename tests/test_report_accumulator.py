from pathlib import Path
s=(Path(__file__).resolve().parents[1] / 'src/paw3222.c').read_text();a=s.index('static void paw32xx_report_work_handler(');b=s.index('static int paw32xx_check_link(',a)
pre=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#include <assert.h>
#include <stdio.h>
#define CONTAINER_OF(p,t,m) ((t*)((char*)(p)-offsetof(t,m)))
#define CLAMP(v,l,h) ((v)<(l)?(l):((v)>(h)?(h):(v)))
#define K_FOREVER 0
#define K_MSEC(v) (v)
#define CONFIG_PAW3222_REPORT_INTERVAL_MS 15
#define INPUT_REL_X 0
#define INPUT_REL_Y 1
struct k_work {int unused;};
struct k_work_delayable {struct k_work work;};
struct k_spinlock {int locked;};
typedef int k_spinlock_key_t;
struct paw32xx_data {void*dev;int suspended;struct k_work_delayable report_work;struct k_spinlock report_lock;int32_t pending_x,pending_y;int report_queue;};
static struct paw32xx_data*during_send;
static int reports,last_x,last_y,inject,scheduled;
static struct k_work_delayable*k_work_delayable_from_work(struct k_work*w){return CONTAINER_OF(w,struct k_work_delayable,work);}
static int atomic_get(int*p){return *p;}
static int k_spin_lock(struct k_spinlock*l){assert(!l->locked);l->locked=1;return 0;}
static void k_spin_unlock(struct k_spinlock*l,int k){assert(l->locked);l->locked=0;}
static int k_work_schedule_for_queue(int*q,struct k_work_delayable*w,int delay){assert(delay==15);scheduled=1;return 0;}
static void paw32xx_queue_motion(struct paw32xx_data*,int16_t,int16_t);
static int input_report_rel(void*dev,int axis,int value,bool sync,int timeout){if(axis==0){assert(!sync);last_x=value;if(inject){inject=0;paw32xx_queue_motion(during_send,7,-8);}}else{assert(sync);last_y=value;reports++;}return 0;}
'''
post=r'''
int main(void){struct paw32xx_data d={0};during_send=&d;
paw32xx_queue_motion(&d,10,-20);paw32xx_queue_motion(&d,-4,5);assert(d.pending_x==6&&d.pending_y==-15);assert(scheduled);
paw32xx_report_work_handler(&d.report_work.work);assert(reports==1&&last_x==6&&last_y==-15);assert(d.pending_x==0&&d.pending_y==0);
paw32xx_report_work_handler(&d.report_work.work);assert(reports==1);
paw32xx_queue_motion(&d,10,1);inject=1;paw32xx_report_work_handler(&d.report_work.work);assert(last_x==10&&last_y==1);assert(d.pending_x==7&&d.pending_y==-8);
paw32xx_report_work_handler(&d.report_work.work);assert(last_x==7&&last_y==-8);
paw32xx_queue_motion(&d,32767,-32768);paw32xx_queue_motion(&d,127,-127);assert(d.pending_x==32767&&d.pending_y==-32768);
d.suspended=1;paw32xx_queue_motion(&d,1,1);int old=reports;paw32xx_report_work_handler(&d.report_work.work);assert(reports==old);
puts("PASS: signed accumulation, zero suppression, acquisition during send, bounded backlog, suspend");}
'''
import subprocess
import tempfile
with tempfile.TemporaryDirectory(prefix='paw3222-test-') as directory:
    source = Path(directory) / 'test.c'
    binary = Path(directory) / 'test'
    source.write_text(pre+s[a:b]+post)
    subprocess.run(['cc', '-std=c11', '-Werror', str(source), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
