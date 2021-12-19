#include "statistics/performance.hpp"

// see: https://stackoverflow.com/questions/23367857/accurate-calculation-of-cpu-usage-given-in-percentage-in-linux?rq=1

void read_cpu_stats(cpu_stat &stats){
    FILE *fp = fopen("/proc/stat","r");
    char cpun[255];
    fscanf(fp,"%s %Lf %Lf %Lf %Lf %Lf %Lf %Lf %Lf", cpun,
                                                    &stats.user, 
                                                    &stats.nice, 
                                                    &stats.system, 
                                                    &stats.idle, 
                                                    &stats.iowait, 
                                                    &stats.irq, 
                                                    &stats.softirq, 
                                                    &stats.steal);
    fclose(fp);
}

static long double cpu_idle(cpu_stat &stats){
    return stats.idle + stats.iowait;
}

static long double cpu_non_idle(cpu_stat &stats){
    return stats.user + stats.nice + stats.system + stats.irq + stats.softirq + stats.steal;
}

static long double cpu_total(cpu_stat &stats){
    return (cpu_idle(stats) + cpu_non_idle(stats));
}

double cpu_usage(cpu_stat &prev_stats, cpu_stat &now_stats){
    long double totald = cpu_total(now_stats) - cpu_total(prev_stats);
    long double idled = cpu_idle(now_stats) - cpu_idle(prev_stats);
    return (totald - idled)/totald;
}

long double networkRead(std::string interface, std::string rt_x){
    long double speed;
    std::string filename =  "/sys/class/net/" + interface + "/statistics/"+ rt_x + "_bytes";
    FILE *fp = fopen(filename.c_str(),"r");
    fscanf(fp,"%Lf",&speed);
    fclose(fp);
    return speed;
}