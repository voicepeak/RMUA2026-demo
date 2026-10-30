// Trace two already-created, cooperatively gated threads of an owned child only.
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <map>
#include <new>
#include <set>
#include <signal.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/ptrace.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/user.h>
#include <sys/wait.h>
#include <thread>
#include <time.h>
#include <unistd.h>

#if !defined(__x86_64__)
#error This diagnostic requires Linux x86-64
#endif
struct Shared {std::atomic<int> ready{0}, gate{0}; std::atomic<pid_t> tids[2]{};};
extern "C" void mock_existing_after_load();
__attribute__((noinline)) void load(const float* p) {
    asm volatile("movups (%0), %%xmm0\n\t"
                 ".global mock_existing_after_load\n"
                 "mock_existing_after_load:\n\t"
                 "nop\n\t" : : "r"(p) : "xmm0", "memory");
}
void worker(Shared* shared,int index) {
    shared->tids[index]=static_cast<pid_t>(syscall(SYS_gettid));
    ++shared->ready;
    while(!shared->gate.load())std::this_thread::yield();
    const float values[4]={index?.9f:.1f,.2f,.3f,.4f};
    for(int i=0;i<3;++i)load(values);
}
long long now_ns() {timespec t{}; if(clock_gettime(CLOCK_MONOTONIC,&t)!=0)return -1; return t.tv_sec*1000000000LL+t.tv_nsec;}
std::set<pid_t> threads(pid_t child) {
    char name[64];std::snprintf(name,sizeof(name),"/proc/%d/task",child);
    DIR* dir=opendir(name);std::set<pid_t> result;
    if(!dir)return result;
    while(auto* entry=readdir(dir)) {int id=0;char extra=0;if(std::sscanf(entry->d_name,"%d%c",&id,&extra)==1)result.insert(id);}
    closedir(dir);return result;
}
bool install(pid_t tid) {
    return ptrace(PTRACE_POKEUSER,tid,offsetof(struct user,u_debugreg[0]),reinterpret_cast<void*>(&mock_existing_after_load))==0
        && ptrace(PTRACE_POKEUSER,tid,offsetof(struct user,u_debugreg[7]),reinterpret_cast<void*>(1))==0;
}
int main() {
    alarm(10);
    void* mem=mmap(nullptr,sizeof(Shared),PROT_READ|PROT_WRITE,MAP_SHARED|MAP_ANONYMOUS,-1,0);
    if(mem==MAP_FAILED)return 2;
    auto* shared=new(mem) Shared();
    const pid_t parent=getpid();
    const pid_t child=fork();
    if(child<0)return 2;
    if(child==0){
        if(prctl(PR_SET_PDEATHSIG,SIGKILL)!=0 || getppid()!=parent)_exit(3);
        std::thread a(worker,shared,0),b(worker,shared,1);a.join();b.join();_exit(0);
    }
    while(shared->ready.load()!=2)std::this_thread::yield();
    const pid_t first=shared->tids[0],second=shared->tids[1];
    const std::set<pid_t> observed=threads(child),expected={child,first,second};
    bool ok=first>0 && second>0 && first!=second && observed==expected;
    int status=0,configured=0,detached=0,hits=0;std::map<pid_t,int> counts;std::map<pid_t,long long> paused;
    for(pid_t tid:{first,second}) {
        if(!ok)break;
        if(ptrace(PTRACE_SEIZE,tid,nullptr,reinterpret_cast<void*>(PTRACE_O_EXITKILL))!=0){ok=false;break;}
        paused[tid]=now_ns();
        if(ptrace(PTRACE_INTERRUPT,tid,nullptr,nullptr)!=0 || waitpid(tid,&status,__WALL)!=tid || !WIFSTOPPED(status)
                || !install(tid)){ok=false;break;}
        ++configured;
    }
    if(ok)for(pid_t tid:{first,second}) {
        if(ptrace(PTRACE_CONT,tid,nullptr,nullptr)!=0){ok=false;break;}
        paused[tid]=now_ns()-paused[tid];
    }
    if(ok)shared->gate=1;
    while(ok && detached<2) {
        const pid_t tid=waitpid(-1,&status,__WALL);
        if((tid!=first && tid!=second) || !WIFSTOPPED(status) || WSTOPSIG(status)!=SIGTRAP){ok=false;break;}
        user_regs_struct regs{};user_fpregs_struct fp{};float values[4]{};
        if(ptrace(PTRACE_GETREGS,tid,nullptr,&regs)!=0 || ptrace(PTRACE_GETFPREGS,tid,nullptr,&fp)!=0
                || regs.rip!=reinterpret_cast<unsigned long long>(&mock_existing_after_load)){ok=false;break;}
        std::memcpy(values,fp.xmm_space,sizeof(values));
        if(std::fabs(values[0]-(tid==first?.1f:.9f))>1e-6f || std::fabs(values[1]-.2f)>1e-6f
                || std::fabs(values[2]-.3f)>1e-6f || std::fabs(values[3]-.4f)>1e-6f){ok=false;break;}
        ++hits;const int count=++counts[tid];
        if(count==3) {
            if(ptrace(PTRACE_POKEUSER,tid,offsetof(struct user,u_debugreg[7]),nullptr)!=0
                    || ptrace(PTRACE_DETACH,tid,nullptr,nullptr)!=0){ok=false;break;}
            ++detached;
        } else if(ptrace(PTRACE_CONT,tid,nullptr,nullptr)!=0){ok=false;break;}
    }
    bool exited=false;
    if(!ok) {
        kill(child,SIGKILL);
        while(waitpid(-1,&status,__WALL)>0){}  // reap only the owned tracee and its attached threads
    } else {
        const pid_t ended=waitpid(child,&status,0);
        exited=ended==child && WIFEXITED(status) && WEXITSTATUS(status)==0;
    }
    ok=ok && exited && configured==2 && detached==2 && hits==6 && counts[first]==3 && counts[second]==3;
    std::printf("{\"enumerated_threads\":%zu,\"expected_threads\":3,\"configured_workers\":%d,"
                "\"detached_workers\":%d,\"hits\":%d,\"hits_per_worker\":[%d,%d],"
                "\"attach_pause_ms\":[%.6f,%.6f],\"owned_child_exited\":%s,\"passed\":%s}\n",
                observed.size(),configured,detached,hits,counts[first],counts[second],paused[first]/1e6,paused[second]/1e6,
                exited?"true":"false",ok?"true":"false");
    munmap(mem,sizeof(Shared));return ok?0:1;
}
