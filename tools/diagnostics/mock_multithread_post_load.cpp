// Trace only a cooperative child and its new threads, never the simulator.
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <map>
#include <signal.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#if !defined(__x86_64__)
#error This diagnostic requires Linux x86-64
#endif
extern "C" void mock_multithread_after_load();
__attribute__((noinline)) void load(const float* p) {
    asm volatile("movups (%0), %%xmm0\n\t"
                 ".global mock_multithread_after_load\n"
                 "mock_multithread_after_load:\n\t"
                 "nop\n\t" : : "r"(p) : "xmm0", "memory");
}
void worker(float first) {
    const float values[4]={first,.2f,.3f,.4f};
    for(int i=0;i<3;++i)load(values);
}
bool breakpoint(pid_t tid) {
    return ptrace(PTRACE_POKEUSER,tid,offsetof(struct user,u_debugreg[0]),
                  reinterpret_cast<void*>(&mock_multithread_after_load))==0
        && ptrace(PTRACE_POKEUSER,tid,offsetof(struct user,u_debugreg[7]),reinterpret_cast<void*>(1))==0;
}
int main(int argc, char** argv) {
    alarm(10);  // tracer failure triggers PTRACE_O_EXITKILL for the owned tracee
    const pid_t child=fork();
    if(child<0)return 2;
    if(child==0) {
        if(ptrace(PTRACE_TRACEME,0,nullptr,nullptr)<0)_exit(3);
        raise(SIGSTOP);
        std::thread a(worker,.1f),b(worker,.9f);
        a.join();b.join();_exit(0);
    }
    int status=0;
    bool ok=waitpid(child,&status,0)==child && WIFSTOPPED(status) && WSTOPSIG(status)==SIGSTOP;
    if(ok)ok=ptrace(PTRACE_SETOPTIONS,child,nullptr,
                     reinterpret_cast<void*>(PTRACE_O_TRACECLONE|PTRACE_O_EXITKILL))==0;
    if(ok && argc==2 && std::strcmp(argv[1], "--exitkill-check")==0) {
        std::printf("{\"owned_child\":%d}\n", child);
        std::fflush(stdout);
        for(;;)pause();  // test kills this owned tracer after reading its child PID
    }
    if(ok)ok=ptrace(PTRACE_CONT,child,nullptr,nullptr)==0;
    std::map<pid_t,int> hit_counts;
    std::map<pid_t,float> values_seen;
    int clones=0, configured=0, total_hits=0;
    bool child_exited=false;
    while(ok && !child_exited) {
        const pid_t tid=waitpid(-1,&status,__WALL);
        if(tid<0){ok=false;break;}
        if(WIFEXITED(status)){if(tid==child){child_exited=true;ok=WEXITSTATUS(status)==0;}continue;}
        if(WIFSIGNALED(status)){ok=false;break;}
        if(!WIFSTOPPED(status)){ok=false;break;}
        const int sig=WSTOPSIG(status);
        if(sig==SIGTRAP && (status>>16)==PTRACE_EVENT_CLONE) {
            unsigned long new_tid=0;
            if(ptrace(PTRACE_GETEVENTMSG,tid,nullptr,&new_tid)!=0 || !new_tid){ok=false;break;}
            ++clones;
        } else if(sig==SIGSTOP && tid!=child) {
            if(!breakpoint(tid)){ok=false;break;}
            ++configured;
        } else if(sig==SIGTRAP && (status>>16)==0) {
            user_regs_struct regs{}; user_fpregs_struct fp{}; float values[4]{};
            if(ptrace(PTRACE_GETREGS,tid,nullptr,&regs)!=0 || ptrace(PTRACE_GETFPREGS,tid,nullptr,&fp)!=0
                    || regs.rip!=reinterpret_cast<unsigned long long>(&mock_multithread_after_load)) {ok=false;break;}
            std::memcpy(values,fp.xmm_space,sizeof(values));
            if((std::fabs(values[0]-.1f)>1e-6f && std::fabs(values[0]-.9f)>1e-6f)
                    || std::fabs(values[1]-.2f)>1e-6f || std::fabs(values[2]-.3f)>1e-6f
                    || std::fabs(values[3]-.4f)>1e-6f){ok=false;break;}
            if(values_seen.count(tid) && values_seen[tid]!=values[0]){ok=false;break;}
            values_seen[tid]=values[0]; ++hit_counts[tid]; ++total_hits;
        } else {ok=false;break;}
        if(ptrace(PTRACE_CONT,tid,nullptr,nullptr)!=0){ok=false;break;}
    }
    if(!ok) {
        kill(child,SIGKILL);
        while(waitpid(-1,&status,__WALL)>0){}
    }
    const bool counts_ok=hit_counts.size()==2 && total_hits==6 && clones==2 && configured==2
        && hit_counts.begin()->second==3 && hit_counts.rbegin()->second==3
        && values_seen.begin()->second!=values_seen.rbegin()->second;
    ok=ok && child_exited && counts_ok;
    std::printf("{\"clones\":%d,\"threads_configured\":%d,\"observed_threads\":%zu,"
                "\"hits\":%d,\"hits_per_thread\":[%d,%d],\"distinct_values\":%s,"
                "\"owned_child_exited\":%s,\"passed\":%s}\n",clones,configured,hit_counts.size(),total_hits,
                hit_counts.empty()?0:hit_counts.begin()->second,hit_counts.size()<2?0:hit_counts.rbegin()->second,
                (values_seen.size()==2 && values_seen.begin()->second!=values_seen.rbegin()->second)?"true":"false",
                child_exited?"true":"false",ok?"true":"false");
    return ok?0:1;
}
