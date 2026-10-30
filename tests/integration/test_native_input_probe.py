"""Native callback forwarding and bounded read using owned mocks only."""
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest

from tests.integration.test_bridge_lifetime_probe import MOCK
from tests.integration.test_bridge_probe import DECLARATIONS


DRIVER = DECLARATIONS + r'''
#include <boost/shared_ptr.hpp>
#include <thread>
namespace ros {struct TimerEvent{};}
using NativeCallback=void(*)(void*,void*);
extern "C" void mock_init(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int)
  asm("_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i");
extern "C" int mock_peak();
extern "C" void owned_native_handler(void*,void*);
extern "C" void mock_native_counts();
int main(){char arg[]="argv";char* argv[]={arg,nullptr};
  mock_init(7,argv,owned_native_handler,true,false,17,"c","d","e",101);
  std::thread update([]{boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>> message;
    airsim_ros::commands_impl::ROTOR_PWM_Callback(message);});
  ros::TimerEvent event;airsim_ros::PublishAsyncCommands(event);update.join();mock_native_counts();return mock_peak();}
'''
SERIAL_DRIVER = DECLARATIONS + r'''
#include <boost/shared_ptr.hpp>
namespace ros {struct TimerEvent{};}
using NativeCallback=void(*)(void*,void*);
extern "C" void mock_init(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int)
  asm("_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i");
extern "C" int mock_peak();
extern "C" void owned_native_handler(void*,void*);
extern "C" void mock_native_counts();
int main(){char arg[]="argv";char* argv[]={arg,nullptr};
  mock_init(7,argv,owned_native_handler,true,false,17,"c","d","e",101);
  ros::TimerEvent event;airsim_ros::PublishAsyncCommands(event);
  boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>> message;
  airsim_ros::commands_impl::ROTOR_PWM_Callback(message);
  mock_native_counts();return mock_peak();}
'''
CONCURRENT_DRIVER = DECLARATIONS + r'''
#include <boost/shared_ptr.hpp>
#include <atomic>
#include <thread>
#include <cstdio>
namespace ros {struct TimerEvent{};}
using NativeCallback=void(*)(void*,void*);
extern "C" void mock_init(int,char**,NativeCallback,bool,bool,int,const char*,const char*,const char*,int)
  asm("_ZN10airsim_ros13InitAirSimROSEiPPcPFvPvS2_EbbNS_17AirSimROSGameTypeEPKcS7_S7_i");
extern "C" void owned_native_handler(void*,void*);
extern "C" void mock_native_counts();
extern "C" int mock_peak();
extern "C" int mock_concurrent_peak();
int main(){char arg[]="argv";char* argv[]={arg,nullptr};
  mock_init(7,argv,owned_native_handler,true,false,17,"c","d","e",101);
  std::atomic<bool> start{false};
  std::thread a([&]{boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>> message;
    while(!start.load()){};for(int i=0;i<32;++i)airsim_ros::commands_impl::ROTOR_PWM_Callback(message);});
  std::thread b([&]{ros::TimerEvent event;while(!start.load()){};
    for(int i=0;i<32;++i)airsim_ros::PublishAsyncCommands(event);});
  start=true;a.join();b.join();mock_native_counts();mock_peak();std::printf("%d\n",mock_concurrent_peak());}
'''

# This mock keeps the parameter alive and deliberately overlaps the original
# functions; it never dereferences freed memory or changes simulator code.
CONCURRENT_MOCK = MOCK.replace('#include <thread>', '''#include <thread>
#include <chrono>
std::atomic<int> active{}, peak{};
struct Activity { Activity(){int n=++active;int p=peak.load();
  while(p<n&&!peak.compare_exchange_weak(p,n)){};
  std::this_thread::sleep_for(std::chrono::milliseconds(2));} ~Activity(){--active;} };
extern "C" int mock_concurrent_peak(){return peak.load();}''')
CONCURRENT_MOCK = CONCURRENT_MOCK.replace(
    'while(phase.load()<(free_in_native()?4:(safe()?2:1)))std::this_thread::yield();\n  free_parameter(&owner);phase=3;',
    'Activity activity; phase=3;')
CONCURRENT_MOCK = CONCURRENT_MOCK.replace('void* parameter=get_parameter(&owner);phase=1;',
                                          'Activity activity; void* parameter=get_parameter(&owner);phase=1;')


class NativeInputTests(unittest.TestCase):
    def test_native_serializer_excludes_concurrent_original_callbacks(self):
        root = Path(__file__).resolve().parents[2]
        build = Path(subprocess.check_output(['bash', str(root/'tools/build_bridge_probe.sh'), 'native-input'],
                                            cwd=root, text=True).strip())
        probe = '/workspace/main/' + str(build.relative_to(root))
        with tempfile.TemporaryDirectory(prefix='native_concurrent_test_', dir=root/'runs') as directory:
            path = Path(directory); inside = '/workspace/main/' + str(path.relative_to(root))
            (path/'mock.cpp').write_text(CONCURRENT_MOCK)
            (path/'driver.cpp').write_text(CONCURRENT_DRIVER)
            def run(*args):
                return subprocess.check_output([str(root/'tools/enter_env.sh'), *args], cwd=root,
                                               text=True, stderr=subprocess.STDOUT, timeout=30)
            run('g++','-std=c++14','-shared','-fPIC','-pthread',inside+'/mock.cpp','-o',inside+'/mock.so')
            run('g++','-std=c++14','-pthread',inside+'/driver.cpp','-L'+inside,
                '-Wl,-rpath,'+inside,'-l:mock.so','-o',inside+'/driver')
            for mode, expected_peak in (('native-input',2),('native-serialize',1)):
                output = run('env','LD_PRELOAD='+probe,'RMUA_BRIDGE_PROBE_ORIGINAL='+inside+'/mock.so',
                             'RMUA_BRIDGE_PROBE_OUTPUT='+inside+'/'+mode+'.jsonl',
                             'RMUA_BRIDGE_PROBE_MODE='+mode,'MOCK_SAFE=1',inside+'/driver')
                self.assertEqual(output.split(), ['1','32','32','0','32','32',str(expected_peak)])
                records = [json.loads(x) for x in (path/(mode+'.jsonl')).read_text().splitlines()]
                entries = [r for r in records if r['event']=='native_pwm_entry']
                self.assertEqual(len(entries),32)
                self.assertTrue(all(r['parameter_read_ok'] and
                                    r['parameter_float32_hex']==struct.pack('<4f',.1,.2,.3,.4).hex()
                                    for r in entries))
                self.assertEqual(records[-1]['overlap_entries'] > 0, mode == 'native-input')
                self.assertEqual(records[-1]['write_errors'], 0)

    def test_native_arguments_results_and_float_bits_survive_and_unreadable_snapshot_does_not_crash(self):
        root = Path(__file__).resolve().parents[2]
        build = Path(subprocess.check_output(['bash', str(root/'tools/build_bridge_probe.sh'), 'native-input'],
                                            cwd=root, text=True).strip())
        probe = '/workspace/main/' + str(build.relative_to(root))
        with tempfile.TemporaryDirectory(prefix='native_input_test_', dir=root/'runs') as directory:
            path = Path(directory)
            inside = '/workspace/main/' + str(path.relative_to(root))
            (path/'mock.cpp').write_text(MOCK)
            (path/'driver.cpp').write_text(DRIVER)

            def run(*args):
                return subprocess.check_output([str(root/'tools/enter_env.sh'), *args], cwd=root,
                                               text=True, stderr=subprocess.STDOUT, timeout=30)

            run('g++','-std=c++14','-shared','-fPIC','-pthread',inside+'/mock.cpp','-o',inside+'/mock.so')
            run('g++','-std=c++14','-pthread',inside+'/driver.cpp','-L'+inside,
                '-Wl,-rpath,'+inside,'-l:mock.so','-o',inside+'/driver')
            (path/'serial_driver.cpp').write_text(SERIAL_DRIVER)
            run('g++','-std=c++14','-pthread',inside+'/serial_driver.cpp','-L'+inside,
                '-Wl,-rpath,'+inside,'-l:mock.so','-o',inside+'/serial_driver')
            for bad_pointer, free_in_native in ((False, False),(True, False),(False, True)):
                name = 'during_native' if free_in_native else ('unreadable' if bad_pointer else 'valid')
                command = ['env','LD_PRELOAD='+probe,'RMUA_BRIDGE_PROBE_ORIGINAL='+inside+'/mock.so',
                           'RMUA_BRIDGE_PROBE_OUTPUT='+inside+'/'+name+'.jsonl',
                           'RMUA_BRIDGE_PROBE_MODE=native-input','MOCK_SAFE=1']
                if bad_pointer:
                    command.append('MOCK_BAD_POINTER=1')
                if free_in_native:
                    command.append('MOCK_FREE_IN_NATIVE=1')
                self.assertEqual(run(*command,inside+'/driver').split(), ['1','1','1','1','1','1'])
                records = [json.loads(x) for x in (path/(name+'.jsonl')).read_text().splitlines()]
                entries = [r for r in records if r['event']=='native_pwm_entry']
                exits = [r for r in records if r['event']=='native_pwm_exit']
                self.assertEqual(len(entries),1)
                self.assertEqual(len(exits),1)
                self.assertTrue(exits[0]['native_call_interval_measured'])
                self.assertEqual(sum(r['event']=='native_callback_bound' for r in records),1)
                locations = [r for r in records if r['event'] == 'native_callback_location']
                self.assertEqual(len(locations), 1)
                self.assertTrue(locations[0]['location_valid'])
                self.assertEqual(locations[0]['bridge_span'], 0)
                self.assertTrue(all(k in locations[0] for k in ('pwm_calls', 'bridge_calls', 'overlap_entries', 'write_errors')))
                from evaluation.metrics.bridge_lifetime import evaluate as lifetime_evaluate
                lifetime_evaluate(records)  # All probe records must satisfy the consumer schema.
                self.assertTrue(bytes.fromhex(locations[0]['module_path_hex']))
                self.assertEqual(int(locations[0]['callback_address'], 16) - int(locations[0]['module_base'], 16),
                                 int(locations[0]['module_offset'], 16))
                self.assertEqual(entries[0]['parameter_read_ok'], not bad_pointer)
                self.assertEqual(entries[0]['bridge_span'], exits[0]['bridge_span'])
                self.assertEqual(records[-1]['write_errors'],0)
                if not bad_pointer:
                    self.assertEqual(entries[0]['parameter_float32_hex'], struct.pack('<4f',.1,.2,.3,.4).hex())
                    from evaluation.metrics.native_input import evaluate
                    start=entries[0]['start_monotonic_ns']*1e-9
                    telemetry=[{'command':{'values':(.1,.2,.3,.4),'sequence':0},
                                'publish_begin_wall_monotonic':start-.001,
                                'publish_end_wall_monotonic':start-.0009}]
                    audit=evaluate(records,telemetry,start+.01)
                    self.assertEqual(audit['parameters_freed_during_original_callback'],int(free_in_native))
            output = run('env','LD_PRELOAD='+probe,'RMUA_BRIDGE_PROBE_ORIGINAL='+inside+'/mock.so',
                         'RMUA_BRIDGE_PROBE_OUTPUT='+inside+'/serial.jsonl',
                         'RMUA_BRIDGE_PROBE_MODE=native-serialize','MOCK_SAFE=1',inside+'/serial_driver')
            self.assertEqual(output.split(), ['1','1','1','1','1','1'])
            serial_records = [json.loads(x) for x in (path/'serial.jsonl').read_text().splitlines()]
            self.assertTrue(any(r['event']=='native_pwm_entry' and r['parameter_read_ok'] for r in serial_records))
            self.assertTrue(all(r['mode']=='native-serialize' for r in serial_records))


if __name__=='__main__':
    unittest.main()
