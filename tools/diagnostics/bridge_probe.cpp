// Local diagnostic interposition for two exported ROS bridge functions only.
// No argument dereferences, physics APIs, game-object queries or publishers.
// Both modes modify timing; neither is eligible for competition acceptance.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace boost { template<class T> class shared_ptr; }
namespace ros { struct TimerEvent; }
namespace airsim_ros {
template<class Allocator> struct RotorPWM_;
namespace commands_impl {
void ROTOR_PWM_Callback(const boost::shared_ptr<const RotorPWM_<std::allocator<void>>>&);
}
void PublishAsyncCommands(const ros::TimerEvent&);
}

namespace {
using Message=boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>>;
using RotorFunction=void(*)(const Message&);
using BridgeFunction=void(*)(const ros::TimerEvent&);
RotorFunction original_rotor=nullptr;
BridgeFunction original_bridge=nullptr;
std::once_flag initialized;
std::recursive_mutex serialized;
std::atomic<unsigned> active[2]{};
std::atomic<long> active_thread[2]{};
std::atomic<std::uint64_t> calls[2]{},overlaps{},write_errors{};
bool locking=false;
int output=-1;

[[noreturn]] void fatal(const char* message) {
    const char prefix[]="RMUA diagnostic bridge probe: ";
    (void)!write(STDERR_FILENO,prefix,sizeof(prefix)-1);
    (void)!write(STDERR_FILENO,message,std::strlen(message));
    (void)!write(STDERR_FILENO,"\n",1);
    _exit(86);
}

std::uint64_t now_ns() {
    timespec value{};
    if(clock_gettime(CLOCK_MONOTONIC,&value)!=0)fatal("monotonic clock unavailable");
    return std::uint64_t(value.tv_sec)*1000000000ULL+value.tv_nsec;
}

void record(const char* event,int function,std::uint64_t begin,std::uint64_t end,long other_thread=0) {
    if(output<0)return;
    char line[768];
    const int length=std::snprintf(line,sizeof(line),
        "{\"event\":\"%s\",\"mode\":\"%s\",\"function\":\"%s\","
        "\"start_monotonic_ns\":%llu,\"end_monotonic_ns\":%llu,\"thread_id\":%ld,\"other_thread_id\":%ld,"
        "\"pwm_calls\":%llu,\"bridge_calls\":%llu,\"overlap_entries\":%llu,\"write_errors\":%llu}\n",
        event,locking?"serialize":"observe",function==0?"pwm":"bridge",
        static_cast<unsigned long long>(begin),static_cast<unsigned long long>(end),syscall(SYS_gettid),other_thread,
        static_cast<unsigned long long>(calls[0].load()),static_cast<unsigned long long>(calls[1].load()),
        static_cast<unsigned long long>(overlaps.load()),static_cast<unsigned long long>(write_errors.load()));
    if(length<=0 || length>=int(sizeof(line)) || write(output,line,length)!=length)++write_errors;
}

void initialize() {
    const char* library=std::getenv("RMUA_BRIDGE_PROBE_ORIGINAL");
    const char* path=std::getenv("RMUA_BRIDGE_PROBE_OUTPUT");
    const char* mode=std::getenv("RMUA_BRIDGE_PROBE_MODE");
    if(!library || !path || !mode)fatal("missing explicit diagnostic configuration");
    if(std::strcmp(mode,"serialize")==0)locking=true;
    else if(std::strcmp(mode,"observe")!=0)fatal("invalid probe mode");
    // Resolve definitions in the already-loaded, unmodified pinned library.
    // Keeping this handle also prevents unloading an in-use original function.
    void* handle=dlopen(library,RTLD_NOW|RTLD_NOLOAD);
    if(!handle)fatal("original bridge library is not loaded");
    original_rotor=reinterpret_cast<RotorFunction>(dlsym(handle,
        "_ZN10airsim_ros13commands_impl18ROTOR_PWM_CallbackERKN5boost10shared_ptrIKNS_9RotorPWM_ISaIvEEEEE"));
    original_bridge=reinterpret_cast<BridgeFunction>(dlsym(handle,
        "_ZN10airsim_ros20PublishAsyncCommandsERKN3ros10TimerEventE"));
    if(!original_rotor || !original_bridge
            || original_rotor==&airsim_ros::commands_impl::ROTOR_PWM_Callback
            || original_bridge==&airsim_ros::PublishAsyncCommands)fatal("original symbols unavailable or recursive");
    output=open(path,O_WRONLY|O_CREAT|O_EXCL|O_APPEND|O_CLOEXEC,0600);
    if(output<0)fatal("exclusive probe output unavailable");
    const auto stamp=now_ns();record("initialized",0,stamp,stamp);
}

class Scope {
    const int index;
    const std::uint64_t begin;
    long other_thread=0;
    bool overlap=false;
public:
    explicit Scope(int value):index(value),begin(now_ns()) {
        const long own_thread=syscall(SYS_gettid);
        active_thread[index]=own_thread;
        ++active[index];++calls[index];
        other_thread=active_thread[1-index].load();
        overlap=active[1-index].load()!=0 && other_thread!=0 && other_thread!=own_thread;
        if(overlap)++overlaps;
    }
    ~Scope() {
        const auto end=now_ns();
        if(--active[index]==0)active_thread[index]=0;
        if(overlap || calls[index].load()==1 || calls[index].load()%128==0)
            record(overlap?"overlap":"periodic",index,begin,end,other_thread);
    }
};

__attribute__((destructor)) void finish() {
    if(output>=0){const auto stamp=now_ns();record("exit",0,stamp,stamp);close(output);output=-1;}
}
}

void airsim_ros::commands_impl::ROTOR_PWM_Callback(const Message& message) {
    std::call_once(initialized,initialize);
    std::unique_lock<std::recursive_mutex> lock(serialized,std::defer_lock);
    if(locking)lock.lock();
    Scope scope(0);
    original_rotor(message);
}

void airsim_ros::PublishAsyncCommands(const ros::TimerEvent& event) {
    std::call_once(initialized,initialize);
    std::unique_lock<std::recursive_mutex> lock(serialized,std::defer_lock);
    if(locking)lock.lock();
    Scope scope(1);
    original_bridge(event);
}
