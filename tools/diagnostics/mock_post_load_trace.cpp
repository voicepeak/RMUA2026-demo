// Isolated ptrace experiment on an owned child. No simulator, ROS or freed memory.
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <signal.h>
#include <sys/ptrace.h>
#include <sys/types.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <unistd.h>

#if !defined(__x86_64__)
#error This diagnostic requires Linux x86-64 debug and SSE registers
#endif

extern "C" void mock_after_load();
__attribute__((noinline)) void consume(const float* p) {
    asm volatile("movups (%0), %%xmm0\n\t"
                 ".global mock_after_load\n"
                 "mock_after_load:\n\t"
                 "nop\n\t" : : "r"(p) : "xmm0", "memory");
}

bool scenario(bool alter) {
    int channel[2];
    if(pipe(channel) != 0)return false;
    const pid_t child=fork();
    if(child < 0){close(channel[0]);close(channel[1]);return false;}
    if(child == 0) {
        close(channel[0]);
        alignas(16) float values[4]={.1f,.2f,.3f,.4f};
        if(write(channel[1],values,sizeof(values)) != sizeof(values))_exit(2);
        close(channel[1]);
        if(ptrace(PTRACE_TRACEME,0,nullptr,nullptr) < 0)_exit(3);
        raise(SIGSTOP);
        if(alter)values[0]=.9f;
        consume(values);
        _exit(0);
    }
    close(channel[1]);
    float snapshot[4]{};
    const auto count=read(channel[0],snapshot,sizeof(snapshot));
    close(channel[0]);
    int status=0;
    const pid_t waited=waitpid(child,&status,0);
    bool ok=count==sizeof(snapshot) && waited==child && WIFSTOPPED(status) && WSTOPSIG(status)==SIGSTOP;
    if(ok)ok=ptrace(PTRACE_POKEUSER,child,offsetof(struct user,u_debugreg[0]),reinterpret_cast<void*>(&mock_after_load))==0;
    if(ok)ok=ptrace(PTRACE_POKEUSER,child,offsetof(struct user,u_debugreg[7]),reinterpret_cast<void*>(1))==0;
    if(ok)ok=ptrace(PTRACE_CONT,child,nullptr,nullptr)==0;
    if(ok)ok=waitpid(child,&status,0)==child && WIFSTOPPED(status) && WSTOPSIG(status)==SIGTRAP;
    user_regs_struct regs{};
    user_fpregs_struct fp{};
    if(ok)ok=ptrace(PTRACE_GETREGS,child,nullptr,&regs)==0 && ptrace(PTRACE_GETFPREGS,child,nullptr,&fp)==0;
    float observed[4]{};
    if(ok) {
        std::memcpy(observed,fp.xmm_space,sizeof(observed));
        ok=regs.rip==reinterpret_cast<unsigned long long>(&mock_after_load)
            && std::fabs(observed[0]-(alter?.9f:.1f))<1e-6f
            && std::fabs(observed[1]-.2f)<1e-6f && std::fabs(observed[2]-.3f)<1e-6f
            && std::fabs(observed[3]-.4f)<1e-6f;
    }
    // The tracee is always reaped; no unrelated process is attached or killed.
    if(waited==child && WIFSTOPPED(status)) {
        if(ok && ptrace(PTRACE_POKEUSER,child,offsetof(struct user,u_debugreg[7]),nullptr)!=0)ok=false;
        if(ok && ptrace(PTRACE_CONT,child,nullptr,nullptr)!=0)ok=false;
        if(!ok)kill(child,SIGKILL);
        if(waitpid(child,&status,0)!=child || !WIFEXITED(status) || WEXITSTATUS(status)!=0)ok=false;
    }
    std::printf("{\"case\":\"%s\",\"snapshot\":[%.9g,%.9g,%.9g,%.9g],"
                "\"loaded_xmm0\":[%.9g,%.9g,%.9g,%.9g],\"post_load_breakpoint_hit\":%s,\"passed\":%s}\n",
                alter?"modified_after_snapshot":"unchanged",snapshot[0],snapshot[1],snapshot[2],snapshot[3],
                observed[0],observed[1],observed[2],observed[3],
                (regs.rip==reinterpret_cast<unsigned long long>(&mock_after_load))?"true":"false",ok?"true":"false");
    return ok;
}
int main() { const bool a=scenario(false), b=scenario(true); return a&&b?0:1; }
