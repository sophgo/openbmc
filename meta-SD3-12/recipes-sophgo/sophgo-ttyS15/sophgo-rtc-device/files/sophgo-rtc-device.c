

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <linux/rtc.h>
#include <time.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
// #define TIME64

int print_flag=0;
int time64_flag=1;

char * queue1_name="/sys/devices/platform/ahb/ahb:apb/1e7e0000.bmc_dev/bmc-dev-queue1";
char * queue2_name="/sys/devices/platform/ahb/ahb:apb/1e7e0000.bmc_dev/bmc-dev-queue2";
char * epoch_name="/sys/class/rtc/rtc0/since_epoch";

void print_log(const char* fmt, ...)
{
    if (print_flag) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
}

#define DIV(a, b) ((a) / (b) - ((a) % (b) < 0))

#define LEAPS_THRU_END_OF(y) (DIV (y, 4) - DIV (y, 100) + DIV (y, 400))

static inline bool is_leap_year(unsigned int year)
{
	return (!(year % 4) && (year % 100)) || !(year % 400);
}

static const unsigned char rtc_days_in_month[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

int rtc_month_days(unsigned int month, unsigned int year)
{
	return rtc_days_in_month[month] + (is_leap_year(year) && month == 1);
}
void rtc_time64_to_tm(__time64_t time, struct rtc_time *tm)
{
	unsigned int month, year, secs;
	int days;
	/* time must be positive */
	// days = div_s64_rem(time, 86400, &secs);
    days = time / 86400;
    secs = time % 86400;
	/* day of the week, 1970-01-01 was a Thursday */
	tm->tm_wday = (days + 4) % 7;
	year = 1970 + days / 365;
	days -= (year - 1970) * 365
		+ LEAPS_THRU_END_OF(year - 1)
		- LEAPS_THRU_END_OF(1970 - 1);
	while (days < 0) {
		year -= 1;
		days += 365 + is_leap_year(year);
	}
	tm->tm_year = year - 1900;
	tm->tm_yday = days + 1;
	for (month = 0; month < 11; month++) {
		int newdays;
		newdays = days - rtc_month_days(month, year);
		if (newdays < 0)
			break;
		days = newdays;
	}
	tm->tm_mon = month;
	tm->tm_mday = days + 1;
	tm->tm_hour = secs / 3600;
	secs -= tm->tm_hour * 3600;
	tm->tm_min = secs / 60;
	tm->tm_sec = secs - tm->tm_min * 60;
	tm->tm_isdst = 0;
}
uint64_t mktime64(const unsigned int year0, const unsigned int mon0,
		const unsigned int day, const unsigned int hour,
		const unsigned int min, const unsigned int sec)
{
	unsigned int mon = mon0, year = year0;

	/* 1..12 -> 11,12,1..10 */
	if (0 >= (int) (mon -= 2)) {
		mon += 12;	/* Puts Feb last since it has leap day */
		year -= 1;
	}

	return ((((uint64_t)
		  (year/4 - year/100 + year/400 + 367*mon/12 + day) +
		  year*365 - 719499
	    )*24 + hour /* now have hours - midnight tomorrow handled here */
	  )*60 + min /* now have minutes */
	)*60 + sec; /* finally seconds */
}
uint64_t rtc_tm_to_time64(struct rtc_time *tm)
{
	return mktime64(((unsigned int)tm->tm_year + 1900), tm->tm_mon + 1,
			tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);
}

int set_rtc_time(struct rtc_time * rtc_time) {
    int fd = open("/dev/rtc", O_RDWR);
    if (fd == -1) {
        perror("Failed to open RTC device");
        return -1;
    }

    if (ioctl(fd, RTC_SET_TIME, rtc_time) == -1) {
        perror("Failed to set RTC time");
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}

int get_rtc_time(struct rtc_time * rtc_time) {
    int fd = open("/dev/rtc", O_RDWR);
    if (fd == -1) {
        perror("Failed to open RTC device");
        return -1;
    }

    if (ioctl(fd, RTC_RD_TIME, rtc_time) == -1) {
        perror("Failed to set RTC time");
        close(fd);
        return 1;
    }

    close(fd);
    return 0;
}


int read_4Byte_from_queue1(char * buf)
{
    int file_fd = open(queue1_name, O_RDONLY);
    read(file_fd, buf, sizeof(int));
    close(file_fd);
}

int write_4Byte_to_queue1(char * buf)
{
    int file_fd = open(queue1_name, O_RDWR);
    write(file_fd, buf, sizeof(int));
    close(file_fd);
}

int read_4Byte_from_queue2(char * buf)
{
    int file_fd = open(queue2_name, O_RDONLY);
    read(file_fd, buf, sizeof(int));
    close(file_fd);
}


int read_epoch_value(uint64_t * epoch_value)
{
    int ret;
    char r_epoch_buf[20];
    int epoch_fd = open(epoch_name, O_RDONLY);
    ret = read(epoch_fd, r_epoch_buf, sizeof(r_epoch_buf));
    * epoch_value = atoll(r_epoch_buf);
    close(epoch_fd);
    return (ret);
}

void * pthread_get_rtc_time(void)
{
    int cmd;
    uint64_t epoch_value = 0;
    char *epoch_buff = (char*)&epoch_value;
    struct rtc_time rtc_time;

    pthread_t tid = 0;
    tid = pthread_self();
    print_log("0-sub tid:%ld in sub thread\n", tid);

     while(1) {
        read_4Byte_from_queue1((char*)&cmd);
        print_log("0-cmd = 0x%x\n",cmd);
        if (cmd == 0x55555555) {
            print_log("0-time: 0x%x\n",time(NULL));

//  Currently, it appears that this two methods are equivalent
#if 0
            read_epoch_value(&epoch_value);
            print_log("0-epoch_value = 0x%llx\n",epoch_value);
            rtc_time64_to_tm(epoch_value, &rtc_time);
            print_log("0-Date : %d-%d-%d %d:%d:%d\n",rtc_time.tm_year+1900,rtc_time.tm_mon+1,rtc_time.tm_mday,rtc_time.tm_hour,rtc_time.tm_min,rtc_time.tm_sec);
#else

            get_rtc_time(&rtc_time);
            print_log("0-Date : %d-%d-%d %d:%d:%d\n",rtc_time.tm_year+1900,rtc_time.tm_mon+1,rtc_time.tm_mday,rtc_time.tm_hour,rtc_time.tm_min,rtc_time.tm_sec);
            epoch_value = rtc_tm_to_time64(&rtc_time);
            print_log("0-epoch_value = 0x%llx\n",epoch_value);
#endif
            write_4Byte_to_queue1((char*)&epoch_buff[0]);
            if (time64_flag)
                write_4Byte_to_queue1((char*)&epoch_buff[4]);

        } else {
            // sleep(1);
        }
     }
}


void * pthread_set_rtc_time(void)
{
    int cmd;
    char r_buf_h[4];
    char r_buf_l[4];
    struct rtc_time rtc_time;
    struct tm tm;
    __time64_t currentTime = 0;
    char * time_buf = (char*)&currentTime;

    pthread_t tid = 0;
    tid = pthread_self();
    print_log("1-sub tid:%ld in sub thread\n", tid);

    while(1) {
        read_4Byte_from_queue2((char*)&cmd);
        print_log("1-cmd = 0x%x\n",cmd);
        if (cmd == 0xaaaaaaaa) {
            read_4Byte_from_queue2((char*)&time_buf[0]);

            if (time64_flag)
                read_4Byte_from_queue2((char*)&time_buf[4]);

            print_log("1-currentTime = 0x%llx\n",currentTime);
            rtc_time64_to_tm(currentTime, &rtc_time);
            print_log("1-Date : %d-%d-%d %d:%d:%d\n",rtc_time.tm_year+1900,rtc_time.tm_mon+1,rtc_time.tm_mday,rtc_time.tm_hour,rtc_time.tm_min,rtc_time.tm_sec);
            set_rtc_time((struct rtc_time *) &rtc_time);
        } else {
            // sleep(1);
        }
     }
}



static const struct option options[] = {
	{ "print_flag",	 required_argument,	0, 'p'},
    { "time64_flag", required_argument,	0, 't'},
	{ 0,  0, 0, 0},
};

int main(int argc, char *argv[])
{
    int ret = 0;
    pthread_t rid;
    pthread_t wid;


    for (;;) {
		int c, idx;

		c = getopt_long(argc, argv, "p:t:", options, &idx);
		if (c == -1)
			break;

		switch (c) {
		case 'p':
			print_flag = atoi(optarg);
			break;
        case 't':
			time64_flag = atoi(optarg);
			break;
		default:
         break;
		}
	}

    ret = pthread_create(&rid, NULL, pthread_get_rtc_time, NULL);

    ret = pthread_create(&wid, NULL, pthread_set_rtc_time, NULL);

    while(1) {
        sleep(5);
    }

}
