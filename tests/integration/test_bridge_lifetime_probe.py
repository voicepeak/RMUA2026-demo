"""Verify lifetime tracing with owned opaque mocks; never read freed storage."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from tests.integration.test_bridge_probe import DECLARATIONS


MOCK = DECLARATIONS + r'''
#include <atomic>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <thread>
struct Owner {void* pointer;};
struct Command {const char* type;void* parameter;};
struct Pwm {float values[4]{.1f,.2f,.3f,.4f};};
Owner owner{std::getenv("MOCK_BAD_POINTER")?reinterpret_cast<void*>(1):static_cast<void*>(new Pwm)};
std::atomic<int> phase{},gets{},frees{},constructs{},handoffs{};
using NativeCallback=void(*)(void*,void*);
NativeCallback native_callback=nullptr;
std::atomic<int> init_calls{},native_calls{};
bool safe(){return std::getenv("MOCK_SAFE")!=nullptr;}
bool free_in_native(){return std::getenv("MOCK_FREE_IN_NATIVE")!=nullptr;}
extern "C" void* get_parameter(const void*) asm("_ZNKSt12__shared_ptrIN10airsim_ros16ROTOR_PWM_ParamsELN9__gnu_cxx12_Lock_policyE2EE3getEv");
extern "C" void* get_command(const void*) asm("_ZNKSt12__shared_ptrIN10airsim_ros7CommandELN9__gnu_cxx12_Lock_policyE2EE3getEv");
extern "C" void* parameter_from_owner(const void*) asm("_ZNSt23_Sp_counted_ptr_inplaceIN10airsim_ros16ROTOR_PWM_ParamsESaIS1_ELN9__gnu_cxx12_Lock_policyE2EE6_M_ptrEv");
extern "C" void free_parameter(void*) asm("_ZNSt23_Sp_counted_ptr_inplaceIN10airsim_ros16ROTOR_PWM_ParamsESaIS1_ELN9__gnu_cxx12_Lock_policyE2EE10_M_destroyEv");
extern "C" void construct1(void*,const char*,void*) asm("_ZN10airsim_ros7CommandC1EPKcPv");
extern "C" void construct2(void*,const char*,void*) asm("_ZN10airsim_ros7CommandC2EPKcPv");
extern "C" void* get_parameter(const void* value){++gets;return static_cast<const Owner*>(value)->pointer;}
extern "C" void* get_command(const void* value){++handoffs;return static_cast<const Owner*>(value)->pointer;}
extern "C" void* parameter_from_owner(const void* value){return static_cast<const Owner*>(value)->pointer;}
extern "C" void free_parameter(void* value){auto p=static_cast<Owner*>(value);if(p->pointer!=reinterpret_cast<void*>(1))delete static_cast<Pwm*>(p->pointer);p->pointer=nullptr;++frees;}
void construct_impl(void* self,const char* type,void* parameter){++constructs;auto c=static_cast<Command*>(self);c->type=type;c->parameter=parameter;}
extern "C" void construct1(void* self,const char* type,void* p){construct_impl(self,type,p);}
extern "C" void construct2(void* self,const char* type,void* p){construct_impl(self,type,p);}
void airsim_ros::commands_impl::ROTOR_PWM_Callback(const boost::shared_ptr<const RotorPWM_<std::allocator<void>>>&){
  while(phase.load()<(free_in_native()?4:(safe()?2:1)))std::this_thread::yield();
  free_parameter(&owner);phase=3;
}
void airsim_ros::PublishAsyncCommands(const ros::TimerEvent&){
  void* parameter=get_parameter(&owner);phase=1;
  if(!safe())while(phase.load()!=3)std::this_thread::yield();
  Command command;
  if(std::getenv("MOCK_CTOR2"))construct2(&command,"PWM",parameter);
  else construct1(&command,"PWM",parameter);
  Owner c{&command};
  void* delivered=get_command(&c);
  if(delivered!=&command || command.parameter!=parameter)std::abort();
  if(native_callback){int result=77;native_callback(delivered,&result);if(result!=78)std::abort();}
  // Deliberately DO NOT dereference parameter: the vulnerable order freed it.
  if(safe())phase=2;
}
extern "C" int mock_peak(){std::printf("%d %d %d %d\n",gets.load(),frees.load(),constructs.load(),handoffs.load());return 0;}
extern "C" void mock_init(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int)
  asm("_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i");
extern "C" void mock_init(int argc,char** argv,NativeCallback cb,bool a,bool b,int game,
                         const char* c,const char* d,const char* e,int f){
  if(argc!=7 || std::strcmp(argv[0],"argv") || !a || b || game!=17 || std::strcmp(c,"c")
      || std::strcmp(d,"d") || std::strcmp(e,"e") || f!=101)std::abort();
  native_callback=cb;++init_calls;
}
extern "C" void owned_native_handler(void* cmd,void* result){
  if(!cmd || !result || std::strcmp(static_cast<Command*>(cmd)->type,"PWM") || *static_cast<int*>(result)!=77)std::abort();
  if(free_in_native()){phase=4;while(phase.load()!=3)std::this_thread::yield();}
  *static_cast<int*>(result)=78;++native_calls;
}
extern "C" void mock_native_counts(){std::printf("%d %d\n",init_calls.load(),native_calls.load());}
'''

DRIVER = DECLARATIONS + r'''
#include <boost/shared_ptr.hpp>
#include <thread>
namespace ros {struct TimerEvent{};}
extern "C" int mock_peak();
int main(){std::thread update([]{boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>> message;
  airsim_ros::commands_impl::ROTOR_PWM_Callback(message);});
  ros::TimerEvent event;airsim_ros::PublishAsyncCommands(event);update.join();return mock_peak();}
'''


class BridgeLifetimeTests(unittest.TestCase):
    def test_free_before_handoff_is_distinct_from_free_after_handoff(self):
        root = Path(__file__).resolve().parents[2]
        build = Path(subprocess.check_output(['bash', str(root / 'tools/build_bridge_probe.sh'), 'lifetime'],
                                            cwd=root, text=True).strip())
        probe = '/workspace/main/' + str(build.relative_to(root))
        with tempfile.TemporaryDirectory(prefix='lifetime_probe_test_', dir=root / 'runs') as directory:
            path = Path(directory)
            inside = '/workspace/main/' + str(path.relative_to(root))
            (path / 'mock.cpp').write_text(MOCK)
            (path / 'driver.cpp').write_text(DRIVER)

            def run(*args):
                return subprocess.check_output([str(root / 'tools/enter_env.sh'), *args],
                                               cwd=root, text=True, stderr=subprocess.STDOUT, timeout=30)

            run('g++', '-std=c++14', '-shared', '-fPIC', '-pthread', inside + '/mock.cpp', '-o', inside + '/mock.so')
            run('g++', '-std=c++14', '-pthread', inside + '/driver.cpp', '-L' + inside,
                '-Wl,-rpath,' + inside, '-l:mock.so', '-o', inside + '/driver')
            for safe, ctor2 in ((False, False), (True, False), (False, True), (True, True)):
                name = ('safe' if safe else 'vulnerable') + ('_ctor2' if ctor2 else '_ctor1')
                environment = ['env', 'LD_PRELOAD=' + probe, 'RMUA_BRIDGE_PROBE_ORIGINAL=' + inside + '/mock.so',
                               'RMUA_BRIDGE_PROBE_OUTPUT=' + inside + '/' + name + '.jsonl',
                               'RMUA_BRIDGE_PROBE_MODE=lifetime']
                if safe:
                    environment.append('MOCK_SAFE=1')
                if ctor2:
                    environment.append('MOCK_CTOR2=1')
                self.assertEqual(run(*environment, inside + '/driver').strip(), '1 1 1 1')
                records = [json.loads(x) for x in (path / (name + '.jsonl')).read_text().splitlines()]
                events = {row['event']: row for row in records}
                self.assertEqual(events['exit']['write_errors'], 0)
                self.assertEqual(events['exit']['pwm_calls'], 1)
                self.assertEqual(events['exit']['bridge_calls'], 1)
                self.assertEqual(events['parameter_borrow']['parameter_address'], events['parameter_free_end']['parameter_address'])
                self.assertEqual(events['parameter_borrow']['parameter_address'], events['command_handoff']['parameter_address'])
                self.assertNotEqual(events['parameter_borrow']['parameter_address'], '0x0')
                self.assertEqual(events['parameter_borrow']['bridge_span'], events['command_handoff']['bridge_span'])
                self.assertEqual(events['command_constructed']['command_address'], events['command_handoff']['command_address'])
                self.assertEqual(events['parameter_borrow']['thread_id'], events['command_handoff']['thread_id'])
                self.assertNotEqual(events['parameter_free_end']['thread_id'], events['command_handoff']['thread_id'])
                self.assertEqual(events['parameter_free_end']['end_monotonic_ns'] < events['command_handoff']['start_monotonic_ns'], not safe)
                from evaluation.metrics.bridge_lifetime import evaluate
                audit = evaluate(records)
                self.assertTrue(audit['hook_coverage_valid'])
                self.assertEqual(audit['freed_parameter_handoffs'], 0 if safe else 1)


if __name__ == '__main__':
    unittest.main()
