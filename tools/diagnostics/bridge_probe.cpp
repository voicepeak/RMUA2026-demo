// Local diagnostic interposition for ROS command interfaces only.
// Default variants pass opaque arguments. Native-input additionally snapshots
// only 16 bytes of identified PWM storage using /proc/self/mem. No game queries,
// physics APIs or publishers. All variants change timing and are diagnostic.
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

#ifdef RMUA_LIFETIME_PROBE
// Exact pinned Itanium ABI symbols. Opaque arguments are forwarded unchanged;
// no payload, shared_ptr layout, Command layout or game object is dereferenced.
extern "C" void* lifetime_get(const void*) asm("_ZNKSt12__shared_ptrIN10airsim_ros16ROTOR_PWM_ParamsELN9__gnu_cxx12_Lock_policyE2EE3getEv");
extern "C" void* lifetime_command_get(const void*) asm("_ZNKSt12__shared_ptrIN10airsim_ros7CommandELN9__gnu_cxx12_Lock_policyE2EE3getEv");
extern "C" void lifetime_free(void*) asm("_ZNSt23_Sp_counted_ptr_inplaceIN10airsim_ros16ROTOR_PWM_ParamsESaIS1_ELN9__gnu_cxx12_Lock_policyE2EE10_M_destroyEv");
extern "C" void lifetime_ctor1(void*,const char*,void*) asm("_ZN10airsim_ros7CommandC1EPKcPv");
extern "C" void lifetime_ctor2(void*,const char*,void*) asm("_ZN10airsim_ros7CommandC2EPKcPv");
#endif
#ifdef RMUA_NATIVE_INPUT_PROBE
using NativeCallback=void(*)(void*,void*);
extern "C" void native_input_init(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int)
    asm("_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i");
#endif

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
const char* mode_name="observe";
int output=-1;
#ifdef RMUA_LIFETIME_PROBE
using Getter=void*(*)(const void*);
using Destroy=void(*)(void*);
using Constructor=void(*)(void*,const char*,void*);
Getter original_get=nullptr,original_command_get=nullptr,original_parameter_from_owner=nullptr;
Destroy original_free=nullptr;
Constructor original_ctor1=nullptr,original_ctor2=nullptr;
struct Context {std::uint64_t span=0;std::uintptr_t parameter=0,command=0;};
thread_local Context context;
std::atomic<std::uint64_t> span_counter{};
#endif
#ifdef RMUA_NATIVE_INPUT_PROBE
using InitFunction=void(*)(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int);
InitFunction original_init=nullptr;
std::atomic<NativeCallback> native_callback{};
int self_memory=-1;
#endif

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

void record(const char* event,int function,std::uint64_t begin,std::uint64_t end,long other_thread=0,
            std::uintptr_t parameter=0,std::uintptr_t command=0,std::uint64_t span=0,
            const char* pwm_hex="",bool read_ok=false,bool call_interval=false) {
    if(output<0)return;
    char line[768];
    const int length=std::snprintf(line,sizeof(line),
        "{\"event\":\"%s\",\"mode\":\"%s\",\"function\":\"%s\","
        "\"start_monotonic_ns\":%llu,\"end_monotonic_ns\":%llu,\"thread_id\":%ld,\"other_thread_id\":%ld,"
        "\"pwm_calls\":%llu,\"bridge_calls\":%llu,\"overlap_entries\":%llu,\"write_errors\":%llu,"
        "\"parameter_address\":\"0x%llx\",\"command_address\":\"0x%llx\",\"bridge_span\":%llu,"
        "\"parameter_read_ok\":%s,\"parameter_float32_hex\":\"%s\",\"native_call_interval_measured\":%s}\n",
        event,mode_name,function==0?"pwm":"bridge",
        static_cast<unsigned long long>(begin),static_cast<unsigned long long>(end),syscall(SYS_gettid),other_thread,
        static_cast<unsigned long long>(calls[0].load()),static_cast<unsigned long long>(calls[1].load()),
        static_cast<unsigned long long>(overlaps.load()),static_cast<unsigned long long>(write_errors.load()),
        static_cast<unsigned long long>(parameter),static_cast<unsigned long long>(command),
        static_cast<unsigned long long>(span),read_ok?"true":"false",pwm_hex,call_interval?"true":"false");
    if(length<=0 || length>=int(sizeof(line)) || write(output,line,length)!=length)++write_errors;
}

void initialize() {
    const char* library=std::getenv("RMUA_BRIDGE_PROBE_ORIGINAL");
    const char* path=std::getenv("RMUA_BRIDGE_PROBE_OUTPUT");
    const char* mode=std::getenv("RMUA_BRIDGE_PROBE_MODE");
    if(!library || !path || !mode)fatal("missing explicit diagnostic configuration");
    mode_name=mode;
#ifdef RMUA_LIFETIME_PROBE
#ifdef RMUA_NATIVE_INPUT_PROBE
    if(std::strcmp(mode,"native-input")!=0)fatal("native input build requires native-input mode");
#else
    if(std::strcmp(mode,"lifetime")!=0)fatal("lifetime build requires lifetime mode");
#endif
#else
    if(std::strcmp(mode,"serialize")==0)locking=true;
    else if(std::strcmp(mode,"observe")!=0)fatal("invalid probe mode");
#endif
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
#ifdef RMUA_LIFETIME_PROBE
    original_get=reinterpret_cast<Getter>(dlsym(handle,"_ZNKSt12__shared_ptrIN10airsim_ros16ROTOR_PWM_ParamsELN9__gnu_cxx12_Lock_policyE2EE3getEv"));
    original_command_get=reinterpret_cast<Getter>(dlsym(handle,"_ZNKSt12__shared_ptrIN10airsim_ros7CommandELN9__gnu_cxx12_Lock_policyE2EE3getEv"));
    original_parameter_from_owner=reinterpret_cast<Getter>(dlsym(handle,"_ZNSt23_Sp_counted_ptr_inplaceIN10airsim_ros16ROTOR_PWM_ParamsESaIS1_ELN9__gnu_cxx12_Lock_policyE2EE6_M_ptrEv"));
    original_free=reinterpret_cast<Destroy>(dlsym(handle,"_ZNSt23_Sp_counted_ptr_inplaceIN10airsim_ros16ROTOR_PWM_ParamsESaIS1_ELN9__gnu_cxx12_Lock_policyE2EE10_M_destroyEv"));
    original_ctor1=reinterpret_cast<Constructor>(dlsym(handle,"_ZN10airsim_ros7CommandC1EPKcPv"));
    original_ctor2=reinterpret_cast<Constructor>(dlsym(handle,"_ZN10airsim_ros7CommandC2EPKcPv"));
    if(!original_get || !original_command_get || !original_parameter_from_owner || !original_free
            || !original_ctor1 || !original_ctor2 || original_get==&lifetime_get
            || original_command_get==&lifetime_command_get || original_free==&lifetime_free
            || original_ctor1==&lifetime_ctor1 || original_ctor2==&lifetime_ctor2)
        fatal("lifetime symbols unavailable or recursive");
#endif
#ifdef RMUA_NATIVE_INPUT_PROBE
    original_init=reinterpret_cast<InitFunction>(dlsym(handle,
        "_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i"));
    if(!original_init || original_init==&native_input_init)fatal("native initialization symbol unavailable");
    self_memory=open("/proc/self/mem",O_RDONLY|O_CLOEXEC);
    if(self_memory<0)fatal("self-memory snapshot unavailable");
#endif
    output=open(path,O_WRONLY|O_CREAT|O_EXCL|O_APPEND|O_CLOEXEC,0600);
    if(output<0)fatal("exclusive probe output unavailable");
    const auto stamp=now_ns();record("initialized",0,stamp,stamp);
}

class Scope {
    const int index;
    const std::uint64_t begin;
    long other_thread=0;
    bool overlap=false;
#ifdef RMUA_LIFETIME_PROBE
    Context previous;
#endif
public:
    explicit Scope(int value):index(value),begin(now_ns()) {
        const long own_thread=syscall(SYS_gettid);
        active_thread[index]=own_thread;
        ++active[index];++calls[index];
        other_thread=active_thread[1-index].load();
        overlap=active[1-index].load()!=0 && other_thread!=0 && other_thread!=own_thread;
        if(overlap)++overlaps;
#ifdef RMUA_LIFETIME_PROBE
        if(index==1){previous=context;context=Context{};context.span=++span_counter;
            record("bridge_begin",1,begin,begin,0,0,0,context.span);}
#endif
    }
    ~Scope() {
        const auto end=now_ns();
#ifdef RMUA_LIFETIME_PROBE
        if(index==1){record("bridge_end",1,end,end,0,context.parameter,context.command,context.span);
            context=previous;}
#endif
        if(--active[index]==0)active_thread[index]=0;
        if(overlap || calls[index].load()==1 || calls[index].load()%128==0)
            record(overlap?"overlap":"periodic",index,begin,end,other_thread);
    }
};

__attribute__((destructor)) void finish() {
    if(output>=0){const auto stamp=now_ns();record("exit",0,stamp,stamp);close(output);output=-1;}
#ifdef RMUA_NATIVE_INPUT_PROBE
    if(self_memory>=0){close(self_memory);self_memory=-1;}
#endif
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

#ifdef RMUA_LIFETIME_PROBE
extern "C" void* lifetime_get(const void* owner) {
    std::call_once(initialized,initialize);
    const auto begin=now_ns();
    void* parameter=original_get(owner);
    if(context.span){context.parameter=reinterpret_cast<std::uintptr_t>(parameter);
        record("parameter_borrow",1,begin,now_ns(),0,context.parameter,0,context.span);}
    return parameter;
}

extern "C" void* lifetime_command_get(const void* owner) {
    std::call_once(initialized,initialize);
    const auto begin=now_ns();
    void* command=original_command_get(owner);
    if(context.span && context.command && reinterpret_cast<std::uintptr_t>(command)==context.command)
        record("command_handoff",1,begin,now_ns(),0,context.parameter,context.command,context.span);
    return command;
}

extern "C" void lifetime_free(void* owner) {
    std::call_once(initialized,initialize);
    // Ask the original type-specific accessor for its parameter address before
    // destruction. Never read the parameter or the owner layout ourselves.
    const auto parameter=reinterpret_cast<std::uintptr_t>(original_parameter_from_owner(owner));
    const auto begin=now_ns();
    record("parameter_free_begin",1,begin,begin,0,parameter,0,context.span);
    original_free(owner);
    record("parameter_free_end",1,begin,now_ns(),0,parameter,0,context.span);
}

namespace {
void construct(Constructor original,void* self,const char* type,void* parameter) {
    const auto begin=now_ns();
    original(self,type,parameter);
    if(context.span && context.parameter && reinterpret_cast<std::uintptr_t>(parameter)==context.parameter){
        context.command=reinterpret_cast<std::uintptr_t>(self);
        record("command_constructed",1,begin,now_ns(),0,context.parameter,context.command,context.span);}
}
}
extern "C" void lifetime_ctor1(void* self,const char* type,void* parameter) {
    std::call_once(initialized,initialize);construct(original_ctor1,self,type,parameter);
}
extern "C" void lifetime_ctor2(void* self,const char* type,void* parameter) {
    std::call_once(initialized,initialize);construct(original_ctor2,self,type,parameter);
}
#endif

#ifdef RMUA_NATIVE_INPUT_PROBE
namespace {
void native_input_callback(void* command,void* result) {
    const auto original=native_callback.load();
    if(!original)fatal("native callback not bound");
    // This TLS context identifies a PWM Command constructed by the original
    // bridge. Unknown commands/sensor callbacks are passed through unread.
    const Context current=context;
    const bool identified=current.span && current.parameter && current.command
        && reinterpret_cast<std::uintptr_t>(command)==current.command;
    if(!identified){original(command,result);return;}
    const auto begin=now_ns();
    unsigned char payload[16]{};
    // Kernel-mediated bounded read avoids dereferencing potentially freed or
    // unmapped storage in this diagnostic. This is a snapshot, not an owning
    // copy, lock, or claim about the consumer's eventual read.
    const bool read_ok=pread(self_memory,payload,sizeof(payload),static_cast<off_t>(current.parameter))==ssize_t(sizeof(payload));
    char encoded[33]{};
    if(read_ok)for(unsigned i=0;i<sizeof(payload);++i)std::snprintf(encoded+2*i,3,"%02x",unsigned(payload[i]));
    record("native_pwm_entry",1,begin,now_ns(),0,current.parameter,current.command,current.span,encoded,read_ok);
    const auto call_begin=now_ns();
    original(command,result);
    const auto end=now_ns();
    record("native_pwm_exit",1,call_begin,end,0,current.parameter,current.command,current.span,"",false,true);
    const auto post_begin=now_ns();
    const bool post_ok=pread(self_memory,payload,sizeof(payload),static_cast<off_t>(current.parameter))==ssize_t(sizeof(payload));
    encoded[0]='\0';
    if(post_ok)for(unsigned i=0;i<sizeof(payload);++i)std::snprintf(encoded+2*i,3,"%02x",unsigned(payload[i]));
    record("native_pwm_post",1,post_begin,now_ns(),0,current.parameter,current.command,current.span,encoded,post_ok);
}
}
extern "C" void native_input_init(int argc,char** argv,NativeCallback callback,bool a,bool b,int game,
                                  const char* c,const char* d,const char* e,int f) {
    std::call_once(initialized,initialize);
    NativeCallback empty=nullptr;
    if(!callback || !native_callback.compare_exchange_strong(empty,callback))fatal("unexpected native callback binding");
    const auto stamp=now_ns();record("native_callback_bound",1,stamp,stamp);
    original_init(argc,argv,native_input_callback,a,b,game,c,d,e,f);
}
#endif
