#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdint.h>
#include <getopt.h>
#include <stdlib.h>
#include <syslog.h>
#include <signal.h>
#include <sys/timerfd.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include "ini.h"

#define DEFAULT_CONF_FILE (PREFIX "/etc/sysmond.conf")
#define MODULE_NAME "sysmon"
#define DEFAULT_INPUT "/sys/class/hwmon/hwmon2/device/in3_input"

#define LOG_INIT(m) do { \
        setlogmask (LOG_UPTO (LOG_NOTICE)); \
	    openlog ((m), LOG_CONS | LOG_PID | LOG_NDELAY, LOG_USER); \
    } while(0)


#define M_LOG(m, a,...) syslog ((LOG_NOTICE),m "_log@[%s: %d]: " a "\n", __FILE__, \
		__LINE__, ##__VA_ARGS__)

#define M_ERROR(m, a,...) syslog ((LOG_ERR),m "_error@[%s: %d]: " a "\n", __FILE__, \
		__LINE__, ##__VA_ARGS__)
		
#define JSON_FMT "{" \
            "\"stamp\": %ld.%ld," \
            "\"battery\": %.3f," \
            "\"battery_percent\": %.3f," \
            "\"battery_max_voltage\": %d," \
            "\"battery_min_voltage\": %d," \
            "\"cpu_temp\": %d," \
            "\"gpu_temp\": %d," \
            "\"cpu_usages\":[%s]," \
            "\"mem_total\": %d," \
            "\"mem_free\": %d," \
            "\"mem_used\": %d," \
            "\"mem_buff_cache\": %d," \
            "\"mem_available\": %d," \
            "\"mem_swap_total\": %d," \
            "\"mem_swap_free\": %d" \
        "}"

#define MAX_BUF 256
#define EQU(a,b) (strncmp(a,b,MAX_BUF) == 0)

typedef struct
{
    char bat_in[MAX_BUF];
    uint16_t max_voltage;
    uint16_t min_voltage;
    uint16_t cutoff_voltage;
    float ratio;
    uint16_t read_voltage;
    float percent;
} sys_bat_t;

typedef struct
{
    char cpu_temp_file[MAX_BUF];
    char gpu_temp_file[MAX_BUF];
    uint16_t cpu;
    uint16_t gpu;
} sys_temp_t;

typedef struct
{
    long int last_idle;
    long int last_sum;
    float percent;
} sys_cpu_t;

typedef struct 
{
    uint32_t m_total;
    uint32_t m_free;
    uint32_t m_available;
    uint32_t m_cache;
    uint32_t m_buffer;
    uint32_t m_swap_total;
    uint32_t m_swap_free;
} sys_mem_t;

typedef struct {
    char conf_file[MAX_BUF];
    char data_file_out[MAX_BUF];
    sys_bat_t bat_stat;
    sys_cpu_t* cpus;
    sys_mem_t mem;
    sys_temp_t temp;
    int n_cpus;
    struct itimerspec sample_period;
    int pwoff_cd;
} app_data_t;

static volatile int running = 1;
static char buf[MAX_BUF];

static void int_handler(int dummy)
{
    (void)dummy;
    running = 0;
}

static void help(const char *app)
{
    fprintf(stderr,
            "Usage: %s options.\n"
            "Options:\n"
            "\t -f <value>: config file\n"
            "\t -h <value>: this help message\n",
            app);
}

static void map(app_data_t* opt)
{
    float volt = opt->bat_stat.read_voltage*opt->bat_stat.ratio;
    if(volt < opt->bat_stat.min_voltage)
    {
        opt->bat_stat.percent = 0.0;
        return;
    }
    float result = 101 - (101 / pow(1 + pow(1.33 * (volt - opt->bat_stat.min_voltage) /
                                          (opt->bat_stat.max_voltage - opt->bat_stat.min_voltage), 4.5), 3));
    if(result > 100.0)
        result = 100.0;
    
    opt->bat_stat.percent = result;
}

static int guard_write(int fd, void* buffer, size_t size)
{
    int n = 0;
    int write_len;
    int st;
    while(n != (int)size)
    {
        write_len = (int)size - n;
        st = write(fd,buffer + n,write_len);
        if(st == -1)
        {
            M_ERROR(MODULE_NAME,"Unable to write to #%d: %s", fd, strerror(errno));
            return -1;
        }
        if(st == 0)
        {
            M_ERROR(MODULE_NAME,"Endpoint %d is closed", fd);
            return -1;
        }
        n += st;
    }
    return n;
}

static int read_line(int fd, char *buf, int size)
{
    int i = 0;
    char c = '\0';
    int n;
    while ((i < size - 1) && (c != '\n'))
    {
        n = read(fd, &c, 1);
        if (n > 0)
        {
            buf[i] = c;
            i++;
        }
        else
            c = '\n';
    }
    buf[i] = '\0';
    return i;
}

static int read_voltage(app_data_t* opts)
{
    int fd, ret;
    fd = open(opts->bat_stat.bat_in, O_RDONLY);
    if(fd < 0)
    {
        M_ERROR(MODULE_NAME, "Unable to open input: %s", opts->bat_stat.bat_in);
        return -1;
    }
    (void)memset(buf, '\0', sizeof(buf));
    ret = read(fd, buf, sizeof(buf));
    if(ret > 0)
    {
        opts->bat_stat.read_voltage = atoi(buf);
        map(opts);
    }
    (void)close(fd);
    return 0;
}

static int read_cpu_info(app_data_t* opts)
{
    int fd, ret, j, i = 0;
    const char d[2] = " ";
	char* token;
	long int sum = 0, idle = 0;
    fd = open("/proc/stat", O_RDONLY);
    if(fd < 0)
    {
        M_ERROR(MODULE_NAME, "Unable to open stat: %s", strerror(errno));
        return -1;
    }
    for (i = 0; i < opts->n_cpus; i++)
    {
        ret = read_line(fd, buf, MAX_BUF);
        if(ret > 0 && buf[0] == 'c' && buf[1] == 'p' && buf[2] == 'u')
        {
            token = strtok(buf,d);
            sum = 0;
            j = 0;
            while(token!=NULL)
            {
                token = strtok(NULL,d);
                if(token!=NULL){
                    sum += atoi(token);
                    if(j==3)
                        idle = atoi(token);
                    j++;
                }
            }
            opts->cpus[i].percent = 100 - (idle-opts->cpus[i].last_idle)*100.0/(sum-opts->cpus[i].last_sum);
            opts->cpus[i].last_idle = idle;
            opts->cpus[i].last_sum = sum;
        }
        else
        {
            M_ERROR(MODULE_NAME, "Unable to read CPU infos at: %d", i);
            break;
        }
    }
    (void) close(fd);
    if(i==0)
    {
        M_ERROR(MODULE_NAME, "No CPU info found");
        return -1;
    }
    return i;
}

static int read_mem_info(app_data_t* opts)
{
    int fd, ret;
    const char d[2] = " ";
    uint32_t data[7];
	char* token;
    fd = open("/proc/meminfo", O_RDONLY);
    if(fd < 0)
    {
        M_ERROR(MODULE_NAME, "Unable to open meminfo: %s", strerror(errno));
        return -1;
    }
    for (int i = 0; i < 7; i++) {
        ret = read_line(fd,buf,MAX_BUF);
        token = strtok(buf,d);
        token = strtok(NULL,d);
        if(token != NULL)
        {
            data[i] = (uint32_t)atoi(token);
        }
        else
        {
            data[i] = 0;
        }
        if(i == 4)
        {
            for (int j = 0; j < 9; j++) {
                ret = read_line(fd,buf,MAX_BUF);
                // skip 10 line
            }
        }
    }
    opts->mem.m_total       = data[0];
    opts->mem.m_free        = data[1];
    opts->mem.m_available   = data[2];
    opts->mem.m_buffer      = data[3];
    opts->mem.m_cache       = data[4];
    opts->mem.m_swap_total  = data[5];
    opts->mem.m_swap_free   = data[6];
    (void)ret;
    (void)close(fd);
    
    
    /*printf("total: %d used: %d, free: %d buffer/cache: %d, available: %d \n",
        opts->mem.m_total / 1024,
        (opts->mem.m_total - opts->mem.m_free - opts->mem.m_buffer-opts->mem.m_cache)/1024,
        opts->mem.m_free/1024,
        (opts->mem.m_buffer+opts->mem.m_cache)/1024,
        opts->mem.m_available/1024);*/
    return 0;
}

static int read_cpu_temp(app_data_t* opts)
{
    int fd, ret;
    if(opts->temp.cpu_temp_file[0] != '\0')
    {
       fd = open(opts->temp.cpu_temp_file, O_RDONLY);
        if(fd < 0)
        {
            M_ERROR(MODULE_NAME, "Unable to open CPU temp file: %s", strerror(errno));
            return -1;
        }
        (void)memset(buf, '\0', sizeof(buf));
        ret = read(fd, buf, MAX_BUF);
        if(ret < 0)
        {
            M_ERROR(MODULE_NAME, "Unable to read CPU temperature: %s", strerror(errno));
            (void) close(fd);
            return -1;
        }
        opts->temp.cpu = (uint16_t)atoi(buf);
        (void) close(fd);
    }
    
    if(opts->temp.gpu_temp_file[0] != '\0')
    {
       fd = open(opts->temp.gpu_temp_file, O_RDONLY);
        if(fd < 0)
        {
            M_ERROR(MODULE_NAME, "Unable to open GPU temp file: %s", strerror(errno));
            return -1;
        }
        (void)memset(buf, '\0', sizeof(buf));
        ret = read(fd, buf, MAX_BUF);
        if(ret < 0)
        {
            M_ERROR(MODULE_NAME, "Unable to read GPU temperature: %s", strerror(errno));
            (void) close(fd);
            return -1;
        }
        opts->temp.gpu = (uint16_t)atoi(buf);
        (void) close(fd);
    }
    return 0;
}

static int log_to_file(app_data_t* opts)
{
    int ret,fd;
    char out_buf[1024];
    if(opts->data_file_out[0] == '\0')
    {
        return 0;
    }
    fd = open(opts->data_file_out, O_CREAT|O_WRONLY|O_APPEND, 0644);
    if(fd < 0)
    {
        M_ERROR(MODULE_NAME, "Unable to open output file: %s", strerror(errno));
        return -1;
    }
    (void)memset(buf,'\0',MAX_BUF);
    char* ptr = buf;
    size_t len = 0;
    for (int i = 0; i < opts->n_cpus; i++) {
        if(MAX_BUF - len -1 <= 0)
        {
            break;
        }
        snprintf(ptr, MAX_BUF - len -1, "%.3f,", opts->cpus[i].percent);
        len = strlen(buf);
        ptr = buf+len;
    }
    buf[len - 1] = '\0';
    struct timeval now;
    gettimeofday(&now, NULL);
    snprintf(out_buf, sizeof(out_buf), JSON_FMT,
        now.tv_sec,
        now.tv_usec,
        opts->bat_stat.read_voltage* opts->bat_stat.ratio,
        opts->bat_stat.percent,
        opts->bat_stat.max_voltage,
        opts->bat_stat.min_voltage,
        opts->temp.cpu,
        opts->temp.gpu,
        buf,
        opts->mem.m_total,
        opts->mem.m_free,
        (opts->mem.m_total - opts->mem.m_free - opts->mem.m_buffer-opts->mem.m_cache),
        opts->mem.m_buffer+opts->mem.m_cache,
        opts->mem.m_available,
        opts->mem.m_swap_total,
        opts->mem.m_swap_free
        );
    ret = guard_write(fd,out_buf,strlen(out_buf));
    if(ret <= 0)
    {
        M_ERROR(MODULE_NAME, "Unable to write data to output file");
        ret = -1;
    }
    if(ret != (int)strlen(out_buf))
    {
        M_ERROR(MODULE_NAME, "Unable to write all battery info to output file");
        ret = -1;
    }
    else
    {
        ret = 0;
    }
    (void) close(fd);
    return ret;
}

static int ini_handle(void *user_data, const char *section, const char *name, const char *value)
{
    (void)section;
    long int period = 0;
    app_data_t* opts = (app_data_t*) user_data;
    if(EQU(name, "battery_max_voltage"))
    {
        opts->bat_stat.max_voltage = atoi(value);
    }
    else if(EQU(name, "battery_min_voltage"))
    {
        opts->bat_stat.min_voltage = atoi(value);
    }
    else if(EQU(name, "battery_cutoff_votalge"))
    {
        opts->bat_stat.cutoff_voltage = atoi(value);
    }
    else if(EQU(name, "battery_divide_ratio"))
    {
        opts->bat_stat.ratio = atof(value);
    }
    else if(EQU(name, "battery_input"))
    {
        strncpy(opts->bat_stat.bat_in, value, MAX_BUF - 1);
    }
    else if(EQU(name, "sample_period"))
    {
        period = atoi(value)*1e6;
        opts->sample_period.it_interval.tv_nsec = period;
        opts->sample_period.it_value.tv_nsec = period;
    }
    else if(EQU(name, "cpu_core_number"))
    {
        opts->n_cpus = atoi(value) + 1;
    }
    else if(EQU(name, "power_off_count_down"))
    {
        opts->pwoff_cd = atoi(value);
    }
    else if(EQU(name, "data_file_out"))
    {
        (void)strncpy(opts->data_file_out, value, MAX_BUF-1);
    }
    else if(EQU(name, "cpu_temperature_input"))
    {
        (void)strncpy(opts->temp.cpu_temp_file, value, MAX_BUF-1);
    }
    else if(EQU(name, "gpu_temperature_input"))
    {
        (void)strncpy(opts->temp.gpu_temp_file, value, MAX_BUF-1);
    }
    else
    {
        M_ERROR(MODULE_NAME, "Ignore unknown configuration %s = %s", name, value);
        return 0;
    }
    return 1;
}

static int load_config(app_data_t* opts)
{
    // global
    (void)memset(opts->data_file_out, '\0', MAX_BUF);
    (void)memset(opts->temp.cpu_temp_file, '\0', MAX_BUF);
    (void)memset(opts->temp.gpu_temp_file, '\0', MAX_BUF);
    opts->pwoff_cd = 5;
    opts->sample_period.it_interval.tv_sec = 0;
    opts->sample_period.it_interval.tv_nsec = 3e+8;
    opts->sample_period.it_value.tv_sec = 0;
    opts->sample_period.it_value.tv_nsec = 3e+8;
    opts->cpus = NULL;
    opts->n_cpus = 2;
    
    //battery
    (void)memset(opts->bat_stat.bat_in, '\0', MAX_BUF);
    opts->bat_stat.max_voltage = 4200;
    opts->bat_stat.min_voltage = 3300;
    opts->bat_stat.cutoff_voltage = 3000;
    opts->bat_stat.ratio = 1.0;
    opts->bat_stat.read_voltage = 0.0;
    opts->bat_stat.percent = 0.0;
    
    (void)memset(&opts->mem, '\0', sizeof(opts->mem));
    (void)memset(&opts->temp, '\0', sizeof(opts->temp));
    
    M_LOG(MODULE_NAME, "Use configuration: %s", opts->conf_file);
    if (ini_parse(opts->conf_file, ini_handle, opts) < 0)
    {
        M_ERROR(MODULE_NAME, "Can't load '%s'", opts->conf_file);
        return -1;
    }
    // check battery configuration
    if((opts->bat_stat.max_voltage < opts->bat_stat.min_voltage) ||
        (opts->bat_stat.max_voltage < opts->bat_stat.cutoff_voltage) ||
        (opts->bat_stat.min_voltage < opts->bat_stat.cutoff_voltage))
    {
        M_ERROR(MODULE_NAME, "Battery configuration is invalid: max: %d, min: %d, cut off: %d",
            opts->bat_stat.max_voltage,
            opts->bat_stat.min_voltage,
            opts->bat_stat.cutoff_voltage);
        return -1;
    }
    return 0;
}

int main(int argc, char *const *argv)
{
    int ret, tfd, count_down;
    float volt;
    uint64_t expirations_count;
    app_data_t opts;
    LOG_INIT(MODULE_NAME);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGABRT, SIG_IGN);
    signal(SIGINT, int_handler);
    (void)strncpy(opts.conf_file, DEFAULT_CONF_FILE, MAX_BUF - 1);
    while ((ret = getopt(argc, argv, "hf:")) != -1)
    {
        switch (ret)
        {
        case 'f':
            (void)strncpy(opts.conf_file, optarg, MAX_BUF-1);
            break;
        default:
            help(argv[0]);
            return -1;
        }
    }
    
    if(optind > argc)
    {
        help(argv[0]);
        return -1;
    }
    
    if(load_config(&opts) != 0)
    {
        fprintf(stderr,"Unable to read config file\n");
        return -1;
    }
    
    M_LOG(MODULE_NAME, "Data Output: %s", opts.data_file_out);
    M_LOG(MODULE_NAME, "Battery input: %s", opts.bat_stat.bat_in);
    M_LOG(MODULE_NAME, "Battery Max voltage: %d", opts.bat_stat.max_voltage);
    M_LOG(MODULE_NAME, "Battery Min voltage: %d", opts.bat_stat.min_voltage);
    M_LOG(MODULE_NAME, "Battery Cut off voltage: %d", opts.bat_stat.cutoff_voltage);
    M_LOG(MODULE_NAME, "Battery Divide ratio: %.3f", opts.bat_stat.ratio);
    M_LOG(MODULE_NAME, "Sample period: %d", (int)(opts.sample_period.it_value.tv_nsec / 1e6));
    M_LOG(MODULE_NAME, "CPU cores: %d", opts.n_cpus);
    M_LOG(MODULE_NAME, "Power off count down: %d", opts.pwoff_cd);
    M_LOG(MODULE_NAME,"CPU temp. input: %s",opts.temp.cpu_temp_file);
    M_LOG(MODULE_NAME,"GPU temp. input: %s",opts.temp.gpu_temp_file);
    
    // init timerfd
    tfd = timerfd_create(CLOCK_MONOTONIC, TFD_CLOEXEC);
    if (tfd == -1)
    {
        M_ERROR(MODULE_NAME, "Unable to create timerfd: %s", strerror(errno));
        fprintf(stderr,"Unable to create timer fd: %s\n", strerror(errno));
        return -1;
    }
    if (timerfd_settime(tfd, 0 /* no flags */, &opts.sample_period, NULL) == -1)
    {
        M_ERROR(MODULE_NAME, "Unable to set framerate period: %s", strerror(errno));
        (void)close(tfd);
        return -1;
    }
    //init CPU monitors
    opts.cpus = (sys_cpu_t*) malloc(opts.n_cpus*sizeof(sys_cpu_t));
    for(int i=0; i < opts.n_cpus; i++)
    {
        opts.cpus[i].last_sum = 0;
        opts.cpus[i].last_idle = 0;
        opts.cpus[i].percent = 0.0;
    }
    // loop
    count_down = opts.pwoff_cd;
    while(running)
    {
        // open the file
        if(read_voltage(&opts) == -1)
        {
            M_ERROR(MODULE_NAME, "Unable to read system voltage");
        }
        volt = opts.bat_stat.read_voltage*opts.bat_stat.ratio;
        if(volt < opts.bat_stat.cutoff_voltage)
        {
            M_LOG(MODULE_NAME, "Invalid voltage read: %.3f", volt);
        }
        else
        {
            if(opts.bat_stat.percent <= 1.0)
            {
                count_down--;
                M_LOG(MODULE_NAME, "Out of battery. Will shutdown after %d count down", count_down);
            }
            else
            {
                // reset the count_down
                count_down = opts.pwoff_cd;
            }
            // check if we should shutdown
            if(count_down <= 0)
            {
                M_LOG(MODULE_NAME, "Shutting down system");
                ret = system("poweroff");
                (void) ret;
                // this should never happend
                return 0;
            }
        }
        // read cpu info
        if(read_cpu_info(&opts) == -1)
        {
            M_ERROR(MODULE_NAME, "Unable to read CPU infos");
        }
        // read memory usage
        if(read_mem_info(&opts) == -1)
        {
            M_ERROR(MODULE_NAME, "Unable to read memory usage");
        }
        // read CPU temperature
        if(read_cpu_temp(&opts) == -1)
        {
            M_ERROR(MODULE_NAME, "Unable to read CPU temperature");
        }
        // TODO read net work trafic
        
        // log to file
        if(log_to_file(&opts) == -1)
        {
            M_ERROR(MODULE_NAME, "Unable to write sysinfo to output");
        }
        // check timeout
        if(read(tfd, &expirations_count, sizeof(expirations_count)) != (int)sizeof(expirations_count))
        {
            M_ERROR(MODULE_NAME, "Unable to read timer: %s", strerror(errno));
        }
        else if (expirations_count > 1u)
        {
            M_ERROR(MODULE_NAME, "LOOP OVERFLOW COUNT: %lu", (long unsigned int)expirations_count);
        }
    }
    
    if(opts.cpus)
        free(opts.cpus);
    
    return 0;
}