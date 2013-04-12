#include <linux/sched.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/kernel_stat.h>
#include <linux/rtc.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/wakelock.h>
#include <../clock.h>
#include <../clock-local.h>
#include "pm.h"
#include <linux/vmalloc.h>
#include <mach/board.h>
#include <mach/rpm.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#include <mach/rpm-8960.h>
#include <mach/rpm-8064.h>
#include <mach/msm_xo.h>
#include <linux/gpio.h>
#include <asm/system_info.h>
#include <linux/tick.h>

#include "board-monarudo.h"
#define HTC_PM_STATSTIC_DELAY			10000

#ifdef arch_idle_time

static cputime64_t get_idle_time(int cpu)
{
	cputime64_t idle;

	idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	if (cpu_online(cpu) && !nr_iowait_cpu(cpu))
		idle += arch_idle_time(cpu);
	return idle;
}

static cputime64_t get_iowait_time(int cpu)
{
	cputime64_t iowait;

	iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	if (cpu_online(cpu) && nr_iowait_cpu(cpu))
		iowait += arch_idle_time(cpu);
	return iowait;
}

#else

static u64 get_idle_time(int cpu)
{
	u64 idle, idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		
		idle = kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE];
	else
		idle = usecs_to_cputime64(idle_time);

	return idle;
}

static u64 get_iowait_time(int cpu)
{
	u64 iowait, iowait_time = get_cpu_iowait_time_us(cpu, NULL);

	if (iowait_time == -1ULL)
		
		iowait = kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT];
	else
		iowait = usecs_to_cputime64(iowait_time);

	return iowait;
}

#endif


extern void htc_print_active_wake_locks(int type);
extern void htc_show_interrupts(void);
extern void htc_timer_stats_onoff(char onoff);
extern void htc_timer_stats_show(u16 water_mark);

static int msm_htc_util_delay_time = HTC_PM_STATSTIC_DELAY;
module_param_named(
	delay_time, msm_htc_util_delay_time, int, S_IRUGO | S_IWUSR | S_IWGRP
);

static struct workqueue_struct *htc_pm_monitor_wq = NULL;
struct delayed_work htc_pm_delayed_work;

static spinlock_t lock;

void htc_xo_vddmin_stat_show(void);
void htc_kernel_top(void);

uint32_t previous_xo_count = 0;
uint64_t previous_xo_time = 0;
uint32_t previous_vddmin_count = 0;
uint64_t previous_vddmin_time = 0;

struct st_htc_idle_statistic {
	u32 count;
	u32 time;
};

struct st_htc_idle_statistic htc_idle_Stat[CONFIG_NR_CPUS][3];

void htc_idle_stat_clear(void)
{
	memset(htc_idle_Stat, 0, sizeof(htc_idle_Stat));
}

void htc_idle_stat_add(int sleep_mode, u32 time)
{
	unsigned int cpu = smp_processor_id();

	if (cpu < CONFIG_NR_CPUS) {
		switch (sleep_mode) {
		case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
			htc_idle_Stat[cpu][0].count++;
			htc_idle_Stat[cpu][0].time += time;
			break;
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE:
			htc_idle_Stat[cpu][1].count++;
			htc_idle_Stat[cpu][1].time += time;
			break;
		case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
			htc_idle_Stat[cpu][2].count++;
			htc_idle_Stat[cpu][2].time += time;
			break;
		default:
			break;
		}
	}
}

void htc_idle_stat_show(u32 total_time)
{
	int i = 0, cpu = 0;
	u32 idle_time = 0;
	total_time *= 1000;

	printk("[K] cpu_id\tcpu_state\tidle_count\tidle_time\n");
	for (cpu = 0; cpu < CONFIG_NR_CPUS; cpu++) {
		for (i = 0; i < 3 ; i++) {
			if (htc_idle_Stat[cpu][i].count) {
				idle_time += htc_idle_Stat[cpu][i].time;
				printk("[K]\t%d\tC%d\t\t%d\t\t%dms\n"
					,cpu , i, htc_idle_Stat[cpu][i].count, htc_idle_Stat[cpu][i].time / 1000);
			}
		}
	}
	htc_xo_vddmin_stat_show();
	msm_rpm_dump_stat();
}

#if 0
static DECLARE_BITMAP(msm_pm_clocks_no_tcxo_shutdown, MAX_NR_CLKS);
#endif
u32 count_xo_block_clk_array[MAX_NR_CLKS] = {0};
void htc_xo_block_clks_count_clear(void)
{
	memset(count_xo_block_clk_array, 0, sizeof(count_xo_block_clk_array));
}
void htc_xo_block_clks_count(void)
{
#if 0
	int ret, i;
	ret = msm_clock_require_tcxo(msm_pm_clocks_no_tcxo_shutdown, MAX_NR_CLKS);
	if (ret) {
		int blk_xo = 0;
		for_each_set_bit(i, msm_pm_clocks_no_tcxo_shutdown, MAX_NR_CLKS) {
			blk_xo = local_clk_src_xo(i);
			if (blk_xo)
				count_xo_block_clk_array[i]++;
		}
	}
#endif
}

void htc_xo_block_clks_count_show(void)
{
#if 0
	int ret, i;
	ret = msm_clock_require_tcxo(msm_pm_clocks_no_tcxo_shutdown, MAX_NR_CLKS);
	if (ret) {
		char clk_name[20] = "\0";
		for_each_set_bit(i, msm_pm_clocks_no_tcxo_shutdown, MAX_NR_CLKS) {
			if (count_xo_block_clk_array[i] > 0) {
				clk_name[0] = '\0';
				ret = msm_clock_get_name_noirq(i, clk_name, sizeof(clk_name));
				pr_info("%s (id=%d): %d\n", clk_name, ret, count_xo_block_clk_array[i]);
			}
		}
	}
#endif
}

void htc_xo_vddmin_stat_show(void)
{
	uint32_t xo_count = 0;
	uint64_t xo_time = 0;
	uint32_t vddmin_count = 0;
	uint64_t vddmin_time = 0;
	if (htc_get_xo_vdd_min_info(&xo_count, &xo_time, &vddmin_count, &vddmin_time)) {
		if (xo_count > previous_xo_count) {
			printk("[K] XO: %u, %llums\n", xo_count-previous_xo_count, xo_time-previous_xo_time);
			previous_xo_count = xo_count;
			previous_xo_time = xo_time;
		}
		if (vddmin_count > previous_vddmin_count) {
			printk("[K] Vdd-min: %u, %llums\n", vddmin_count-previous_vddmin_count, vddmin_time-previous_vddmin_time);
			previous_vddmin_count = vddmin_count;
			previous_vddmin_time = vddmin_time;
		}
	}
}

void htc_print_vddmin_gpio_status(void)
{
	if (system_rev == XB || system_rev == XC)
		printk(KERN_INFO "[K] AP2MDM_VDDMIN: %d, MDM2AP_VDDMIN: %d. \n", gpio_get_value(AP2MDM_VDDMIN), gpio_get_value(MDM2AP_VDDMIN));
}

void htc_pm_monitor_work(struct work_struct *work)
{
	struct timespec ts;
	struct rtc_time tm;

	if (htc_pm_monitor_wq == NULL){
		printk(KERN_INFO "[K] hTc PM statistic is NILL.\n");
		return;
	}

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec - (sys_tz.tz_minuteswest * 60), &tm);
	printk("[K] [PM] hTC PM Statistic start (%02d-%02d %02d:%02d:%02d) \n",
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	htc_show_interrupts();
	htc_xo_block_clks_count_show();
	htc_xo_block_clks_count_clear();
	msm_xo_print_voters();
	htc_idle_stat_show(msm_htc_util_delay_time);
	htc_idle_stat_clear();
	htc_timer_stats_onoff('0');
	htc_timer_stats_show(300);
	htc_timer_stats_onoff('1');
#ifdef CONFIG_PERFLOCK
	htc_print_active_perf_locks();
#endif
	
	
	htc_print_active_wake_locks(WAKE_LOCK_SUSPEND);
	htc_print_vddmin_gpio_status();

	queue_delayed_work(htc_pm_monitor_wq, &htc_pm_delayed_work, msecs_to_jiffies(msm_htc_util_delay_time));
	htc_kernel_top();
	printk("[K] [PM] hTC PM Statistic done\n");
}

static u32 full_loading_counter = 0;

#define MAX_PID 32768
#define NUM_BUSY_THREAD_CHECK 5
unsigned int *prev_proc_stat = NULL;
int *curr_proc_delta = NULL;
struct task_struct **task_ptr_array = NULL;
struct kernel_cpustat new_cpu_stat, old_cpu_stat;

int findBiggestInRange(int *array, int max_limit_idx)
{
	int largest_idx = 0, i;

	for (i = 0 ; i < MAX_PID ; i++) {
		if (array[i] > array[largest_idx] && (max_limit_idx == -1 || array[i] < array[max_limit_idx]))
			largest_idx = i;
	}

	return largest_idx;
}

void sorting(int *source, int *output)
{
	int i;
	for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		if (i == 0)
			output[i] = findBiggestInRange(source, -1);
		else
			output[i] = findBiggestInRange(source, output[i-1]);
	}
}

static void get_all_cpu_stat(struct kernel_cpustat *cpu_stat)
{
	int i;

	if (!cpu_stat)
		return;
	memset(cpu_stat, 0, sizeof(struct kernel_cpustat));

	for_each_possible_cpu(i) {
		cpu_stat->cpustat[CPUTIME_USER] += kcpustat_cpu(i).cpustat[CPUTIME_USER];
		cpu_stat->cpustat[CPUTIME_NICE] += kcpustat_cpu(i).cpustat[CPUTIME_NICE];
		cpu_stat->cpustat[CPUTIME_SYSTEM] += kcpustat_cpu(i).cpustat[CPUTIME_SYSTEM];
		cpu_stat->cpustat[CPUTIME_IDLE] += get_idle_time(i);
		cpu_stat->cpustat[CPUTIME_IOWAIT] += get_iowait_time(i);
		cpu_stat->cpustat[CPUTIME_IRQ] += kcpustat_cpu(i).cpustat[CPUTIME_IRQ];
		cpu_stat->cpustat[CPUTIME_SOFTIRQ] += kcpustat_cpu(i).cpustat[CPUTIME_SOFTIRQ];
		cpu_stat->cpustat[CPUTIME_STEAL] += kcpustat_cpu(i).cpustat[CPUTIME_STEAL];
		cpu_stat->cpustat[CPUTIME_GUEST] += kcpustat_cpu(i).cpustat[CPUTIME_GUEST];
		cpu_stat->cpustat[CPUTIME_GUEST_NICE] += kcpustat_cpu(i).cpustat[CPUTIME_GUEST_NICE];
	}
}

void htc_kernel_top(void)
{
	struct task_struct *p;
	int top_loading[NUM_BUSY_THREAD_CHECK], i;
	unsigned long user_time, system_time, io_time;
	unsigned long irq_time, idle_time, delta_time;
	ulong flags;
	struct task_cputime cputime;
	int dump_top_stack = 0;

	if (task_ptr_array == NULL ||
			curr_proc_delta == NULL ||
			prev_proc_stat == NULL)
		return;

	spin_lock_irqsave(&lock, flags);
	get_all_cpu_stat(&new_cpu_stat);

	
	for_each_process(p) {
		thread_group_cputime(p, &cputime);

		if (p->pid < MAX_PID) {
			curr_proc_delta[p->pid] =
				(cputime.utime + cputime.stime)
				- (prev_proc_stat[p->pid]);
			task_ptr_array[p->pid] = p;
		}
	}

	
	sorting(curr_proc_delta, top_loading);

	
	user_time = (unsigned long)((new_cpu_stat.cpustat[CPUTIME_USER] + new_cpu_stat.cpustat[CPUTIME_NICE])
			- (old_cpu_stat.cpustat[CPUTIME_USER] + old_cpu_stat.cpustat[CPUTIME_NICE]));
	system_time = (unsigned long)(new_cpu_stat.cpustat[CPUTIME_SYSTEM] - old_cpu_stat.cpustat[CPUTIME_SYSTEM]);
	io_time = (unsigned long)(new_cpu_stat.cpustat[CPUTIME_IOWAIT] - old_cpu_stat.cpustat[CPUTIME_IOWAIT]);
	irq_time = (unsigned long)((new_cpu_stat.cpustat[CPUTIME_IRQ] + new_cpu_stat.cpustat[CPUTIME_SOFTIRQ])
			- (old_cpu_stat.cpustat[CPUTIME_IRQ] + old_cpu_stat.cpustat[CPUTIME_SOFTIRQ]));
	idle_time = (unsigned long)
	((new_cpu_stat.cpustat[CPUTIME_IDLE] + new_cpu_stat.cpustat[CPUTIME_STEAL] + new_cpu_stat.cpustat[CPUTIME_GUEST])
	 - (old_cpu_stat.cpustat[CPUTIME_IDLE] + old_cpu_stat.cpustat[CPUTIME_STEAL] + old_cpu_stat.cpustat[CPUTIME_GUEST]));
	delta_time = user_time + system_time + io_time + irq_time + idle_time;

	if ((full_loading_counter >= 9) && (full_loading_counter % 3 == 0))
		 dump_top_stack = 1;

	
	printk("[K] CPU Usage\t\tPID\t\tName\n");
	for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		printk("[K] %8lu%%\t\t%d\t\t%s\t\t%d\n",
				curr_proc_delta[top_loading[i]] * 100 / delta_time,
				top_loading[i],
				task_ptr_array[top_loading[i]]->comm,
				curr_proc_delta[top_loading[i]]);
	}

	
	if (dump_top_stack) {
	   struct task_struct *t;
	   for (i = 0 ; i < NUM_BUSY_THREAD_CHECK ; i++) {
		if (task_ptr_array[top_loading[i]] != NULL && task_ptr_array[top_loading[i]]->stime > 0) {
			t = task_ptr_array[top_loading[i]];
			
			do {
				printk("\n[K] ###pid:%d name:%s state:%lu ppid:%d stime:%lu utime:%lu\n",
				t->pid, t->comm, t->state, t->real_parent->pid, t->stime, t->utime);
				show_stack(t, t->stack);
				t = next_thread(t);
			} while (t != task_ptr_array[top_loading[i]]);
		}
	   }
	}
	
	for_each_process(p) {
		if (p->pid < MAX_PID) {
			thread_group_cputime(p, &cputime);
			prev_proc_stat[p->pid] = cputime.stime + cputime.utime;
		}
	}

	old_cpu_stat = new_cpu_stat;

	memset(curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(task_ptr_array, 0, sizeof(int) * MAX_PID);

	spin_unlock_irqrestore(&lock, flags);
}

void htc_pm_monitor_init(void)
{
	if (htc_pm_monitor_wq == NULL)
		return;

	queue_delayed_work(htc_pm_monitor_wq, &htc_pm_delayed_work, msecs_to_jiffies(msm_htc_util_delay_time));

	spin_lock_init(&lock);

	prev_proc_stat = vmalloc(sizeof(int) * MAX_PID);
	curr_proc_delta = vmalloc(sizeof(int) * MAX_PID);
	task_ptr_array = vmalloc(sizeof(int) * MAX_PID);

	memset(prev_proc_stat, 0, sizeof(int) * MAX_PID);
	memset(curr_proc_delta, 0, sizeof(int) * MAX_PID);
	memset(task_ptr_array, 0, sizeof(int) * MAX_PID);

	get_all_cpu_stat(&new_cpu_stat);
	get_all_cpu_stat(&old_cpu_stat);

}



void htc_monitor_init(void)
{
	if (htc_pm_monitor_wq == NULL) {
		
		htc_pm_monitor_wq = create_workqueue("htc_pm_monitor_wq");
		printk(KERN_INFO "[K] Create HTC private workqueue(0x%x)...\n", (unsigned int)htc_pm_monitor_wq);
	}

	if (htc_pm_monitor_wq){
		printk(KERN_INFO "[K] htc_pm_monitor_wq is not NULL.\n");
		INIT_DELAYED_WORK(&htc_pm_delayed_work, htc_pm_monitor_work);
	}
}

