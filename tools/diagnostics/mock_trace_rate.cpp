// Owned-child 200 Hz tracing overhead experiment. No simulator or ROS access.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <new>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#if !defined(__x86_64__)
#error This diagnostic requires Linux x86-64
#endif
constexpr int SAMPLES=200;
constexpr long long PERIOD_NS=5000000;
struct Row {long long target,begin,end;};
struct Shared {Row rows[SAMPLES]{};int written=0;};
long long now_ns(){timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)_exit(9);return t.tv_sec*1000000000LL+t.tv_nsec;}
extern "C" void mock_rate_after_load();
__attribute__((noinline)) void load(const float* p) {
    asm volatile("movups (%0), %%xmm0\n\t"
                 ".global mock_rate_after_load\n"
                 "mock_rate_after_load:\n\t"
                 "nop\n\t" : : "r"(p) : "xmm0", "memory");
}
static bool run(bool traced) {
    void* mem=mmap(nullptr,sizeof(Shared),PROT_READ|PROT_WRITE,MAP_SHARED|MAP_ANONYMOUS,-1,0);
    if(mem==MAP_FAILED)return false;
    auto* shared=new(mem) Shared();
    const pid_t parent=getpid();
    const pid_t child=fork();
    if(child<0){munmap(mem,sizeof(Shared));return false;}
    if(child==0) {
        if(prctl(PR_SET_PDEATHSIG,SIGKILL)!=0 || getppid()!=parent)_exit(3);
        if(traced) {
            if(ptrace(PTRACE_TRACEME,0,nullptr,nullptr)!=0)_exit(4);
            raise(SIGSTOP);
        }
        const float values[4]={.1f,.2f,.3f,.4f};
        const long long first=now_ns()+20000000;
        for(int i=0;i<SAMPLES;++i) {
            const long long target=first+i*PERIOD_NS;
            const timespec deadline{target/1000000000LL,target%1000000000LL};
            if(clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&deadline,nullptr)!=0)_exit(5);
            shared->rows[i].target=target;shared->rows[i].begin=now_ns();
            load(values);
            shared->rows[i].end=now_ns();shared->written=i+1;
        }
        _exit(0);
    }
    int status=0,hits=0;bool ok=true,exited=false;
    if(traced) {
        ok=waitpid(child,&status,0)==child && WIFSTOPPED(status) && WSTOPSIG(status)==SIGSTOP;
        if(ok)ok=ptrace(PTRACE_SETOPTIONS,child,nullptr,reinterpret_cast<void*>(PTRACE_O_EXITKILL))==0;
        if(ok)ok=ptrace(PTRACE_POKEUSER,child,offsetof(struct user,u_debugreg[0]),reinterpret_cast<void*>(&mock_rate_after_load))==0;
        if(ok)ok=ptrace(PTRACE_POKEUSER,child,offsetof(struct user,u_debugreg[7]),reinterpret_cast<void*>(1))==0;
        if(ok)ok=ptrace(PTRACE_CONT,child,nullptr,nullptr)==0;
        while(ok && !exited) {
            if(waitpid(child,&status,0)!=child){ok=false;break;}
            if(WIFEXITED(status)){exited=true;ok=WEXITSTATUS(status)==0;break;}
            if(!WIFSTOPPED(status)||WSTOPSIG(status)!=SIGTRAP){ok=false;break;}
            user_regs_struct regs{};user_fpregs_struct fp{};float values[4]{};
            if(ptrace(PTRACE_GETREGS,child,nullptr,&regs)!=0 || ptrace(PTRACE_GETFPREGS,child,nullptr,&fp)!=0
                    || regs.rip!=reinterpret_cast<unsigned long long>(&mock_rate_after_load)){ok=false;break;}
            std::memcpy(values,fp.xmm_space,sizeof(values));
            if(std::fabs(values[0]-.1f)>1e-6f || std::fabs(values[3]-.4f)>1e-6f){ok=false;break;}
            ++hits;
            if(ptrace(PTRACE_CONT,child,nullptr,nullptr)!=0){ok=false;break;}
        }
    } else {
        const pid_t done=waitpid(child,&status,0);
        exited=done==child && WIFEXITED(status) && WEXITSTATUS(status)==0;
        ok=exited;
    }
    if(!exited){kill(child,SIGKILL);while(waitpid(child,&status,__WALL)>0){if(WIFEXITED(status)||WIFSIGNALED(status))break;}}
    ok=ok && exited && shared->written==SAMPLES && (!traced || hits==SAMPLES);
    std::printf("{\"case\":\"%s\",\"samples\":%d,\"hits\":%d,\"period_ns\":%lld,\"passed\":%s,\"rows\":[",
                traced?"traced":"baseline",shared->written,hits,PERIOD_NS,ok?"true":"false");
    for(int i=0;i<shared->written && i<SAMPLES;++i){
        const Row& row=shared->rows[i];
        std::printf("%s[%lld,%lld]",i?",":"",row.end-row.begin,row.begin-row.target);
    }
    std::printf("]}\n");munmap(mem,sizeof(Shared));return ok;
}
int main(){alarm(12);const bool a=run(false),b=run(true);return a&&b?0:1;}
