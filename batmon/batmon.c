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
#include <time.h>
#include <math.h>

#define MODULE_NAME "batmon"
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
            "\"stamp\": %ld," \
            "\"battery\": %.2f," \
            "\"battery_percent\": %.2f," \
            "\"max_voltage\": %d," \
            "\"min_voltage\": %d" \
        "}\n"

#define MAX_BUF 256
#define SLEEP_TIME 500000 // usec

typedef struct
{
    char hwmon_in[MAX_BUF];
    uint16_t max_voltage;
    uint16_t min_voltage;
    uint16_t cutoff_voltage;
    float ratio;
    char bat_log[MAX_BUF];
    uint16_t read_voltage;
} app_opt_t;

static volatile int running = 1;

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
            "\t -i <value>: input hwmon file\n"
            "\t -o <value>: output battery value to file\n"
            "\t -u <value>: Max battery voltage\n"
            "\t -l <value>: Min battery voltage\n"
            "\t -c <value>: Cut off voltage\n"
            "\t -r <value>: Voltage divide ratio\n",
            app);
}

static float map(app_opt_t* opt)
{
    float volt = opt->read_voltage*opt->ratio;
    if(volt < opt->min_voltage)
        return 0.0;
    float result = 101 - (101 / pow(1 + pow(1.33 * (volt - opt->min_voltage) /
                                          (opt->max_voltage - opt->min_voltage), 4.5), 3));
    if(result > 100.0)
        return 100.0;
    return result;
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

int main(int argc, char *const *argv)
{
    int ret, fd, fdo;
    float last_percent, percent;
    float volt;
    char buf[MAX_BUF];
    app_opt_t opt;
    LOG_INIT(MODULE_NAME);
    signal(SIGPIPE, SIG_IGN);
    signal(SIGABRT, SIG_IGN);
    signal(SIGINT, int_handler);
    (void)memset(opt.hwmon_in, '\0', MAX_BUF);
    (void)memset(opt.bat_log, '\0', MAX_BUF);
    opt.max_voltage = 12600;
    opt.min_voltage = 10000;
    opt.cutoff_voltage = 9000;
    opt.ratio = 3.36;
    last_percent = 0.0;
    opt.read_voltage = 0.0;
    percent = 0.0;
    (void)strncpy(opt.hwmon_in, DEFAULT_INPUT, MAX_BUF);
    while ((ret = getopt(argc, argv, "i:o:u:l:c:r:")) != -1)
    {
        switch (ret)
        {
        case 'i':
            (void)strncpy(opt.hwmon_in, optarg, MAX_BUF);
            break;
        case 'o':
            (void)strncpy(opt.bat_log, optarg, MAX_BUF);
            break;
            
        case 'u':
            opt.max_voltage = (uint16_t)atoi(optarg);
            break;
 
        case 'l':
            opt.min_voltage = (uint16_t)atoi(optarg);
            break;
 
        case 'c':
            opt.cutoff_voltage = (uint16_t)atoi(optarg);
            break;
        case 'r':
            opt.ratio = atof(optarg);
            break;
        default:
            help(argv[0]);
            return -1;
        }
    }
    
    if ((optind > argc) ||
        (opt.max_voltage < opt.min_voltage) ||
        (opt.max_voltage < opt.cutoff_voltage) ||
        (opt.cutoff_voltage > opt.min_voltage))
    {
        help(argv[0]);
        return -1;
    }
    
    M_LOG(MODULE_NAME, "Input: %s", opt.hwmon_in);
    M_LOG(MODULE_NAME, "Output: %s", opt.bat_log);
    M_LOG(MODULE_NAME, "Max voltage: %d", opt.max_voltage);
    M_LOG(MODULE_NAME, "Min voltage: %d", opt.min_voltage);
    M_LOG(MODULE_NAME, "Cut off voltage: %d", opt.cutoff_voltage);
    M_LOG(MODULE_NAME, "Divide ratio: %.3f", opt.ratio);
    
    while(running)
    {
        // open the file
        fd = open(opt.hwmon_in, O_RDONLY);
        if(fd < 0)
        {
            M_ERROR(MODULE_NAME, "Unable to open input: %s", opt.hwmon_in);
            return -1;
        }
        (void)memset(buf, '\0', sizeof(buf));
        ret = read(fd, buf, sizeof(buf));
        if(ret > 0)
        {
            opt.read_voltage = atoi(buf);
            volt = opt.read_voltage*opt.ratio;
            if(volt < opt.cutoff_voltage)
            {
                // Invalid voltage read
                M_LOG(MODULE_NAME, "Invalid voltage read: %d", opt.read_voltage);
                // wait ?
            }
            else
            {
                percent = map(&opt);
                if(percent - last_percent > 5)
                {
                    M_LOG(MODULE_NAME, "Battery percent: %.3f", percent);
                    last_percent = percent;
                }
                if(percent <= 1.0)
                {
                    // shutdown the system
                    M_LOG(MODULE_NAME, "Out of battery. Shutdown");
                    (void) system("poweroff");
                    // this should not happend
                }
                if(opt.bat_log[0] != '\0')
                {
                    fdo = open(opt.bat_log,O_WRONLY | O_APPEND);
                    if(fdo < 0)
                    {
                        //M_ERROR(MODULE_NAME, "Unable to open output file. Battery output is disabled");
                    }
                    else
                    {
                        (void)snprintf(buf,MAX_BUF,JSON_FMT,
                            time(NULL),
                            volt,
                            percent,
                            opt.max_voltage,
                            opt.min_voltage);
                        ret = guard_write(fdo,buf,strlen(buf));
                        if(ret <= 0)
                        {
                            M_ERROR(MODULE_NAME, "Unable to write data to output file");
                        }
                        if(ret != strlen(buf))
                        {
                            M_ERROR(MODULE_NAME, "Unable to write all battery info to output file");
                        }
                        (void)close(fdo);
                        fdo = -1;
                    }
                }
            }
        }
        
        (void)close(fd);
        fd = -1;
        fdo = -1;
        usleep(SLEEP_TIME);
    }
    
    if(fd  > 0)
        (void)close(fd);
    if(fdo > 0)
        (void)close(fdo);
    
    return 0;
}