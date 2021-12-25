#include "statistics/performance.hpp"
#include <sstream>

std::string to_string_with_precision(float value, const int n = 2){
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << value;
    return out.str();
}

// see: https://stackoverflow.com/questions/23367857/accurate-calculation-of-cpu-usage-given-in-percentage-in-linux?rq=1



// =============================================================================================
//                                            CPU USAGE
// =============================================================================================

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

double cpu_usage_get(cpu_stat &prev_stats, cpu_stat &now_stats){
    long double totald = cpu_total(now_stats) - cpu_total(prev_stats);
    long double idled = cpu_idle(now_stats) - cpu_idle(prev_stats);
    return (totald - idled)/totald;
}

static cpu_stat preStats;
void get_cpu_usage(cpu_usage &cu){    
    cpu_stat nowStats;
    read_cpu_stats(nowStats); 
    cu.used = cpu_usage_get(preStats, nowStats);
    memcpy(&preStats, &nowStats, sizeof(cpu_stat));
}

// =============================================================================================
//                                             RAM USAGE
// =============================================================================================

ram_usage get_ram_usage(){
    ram_usage ru;
    static struct sysinfo system_info;
    sysinfo(&system_info);
    ru.used = (system_info.totalram - system_info.freeram);
    ru.total = system_info.totalram;
    ru.free = system_info.freeram;
    return ru;
}


// =============================================================================================
//                                             NETWORK
// =============================================================================================

static net_usage prevNet;

long double networkRead(std::string interface, std::string rt_x){
    long double speed;
    std::string filename =  "/sys/class/net/" + interface + "/statistics/"+ rt_x + "_bytes";
    FILE *fp = fopen(filename.c_str(),"r");
    fscanf(fp,"%Lf",&speed);
    fclose(fp);
    return speed;
}

void get_net_stat(net_usage &stats){
    double netnowUp = networkRead(NET_INTERFACE, "tx"); 
    double netnowDown = networkRead(NET_INTERFACE, "rx");  
    stats.up = netnowUp - prevNet.up;
    stats.down = netnowDown -  prevNet.down;
    prevNet.up = netnowUp;
    prevNet.down = netnowDown;
    prevNet.last_read = clock();
}

net_usage get_net_usage(){
    net_usage nu;
    double timeFromLast = (clock() - prevNet.last_read) / (double) CLOCKS_PER_SEC;
    if(timeFromLast >= 1.0){
        net_usage nowNet;
        get_net_stat(nowNet);
        nu.up = nowNet.up;
        nu.down = nowNet.down;
    } else {
        nu.up = prevNet.up;
        nu.down = prevNet.down;
    }
    return nu;
}