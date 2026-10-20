"""Native probe forwarding/concurrency test using owned mock libraries only."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest


DECLARATIONS=r'''
#include <memory>
namespace boost {template<class T> class shared_ptr;}
namespace ros {struct TimerEvent;}
namespace airsim_ros {
template<class A> struct RotorPWM_;
namespace commands_impl {void ROTOR_PWM_Callback(const boost::shared_ptr<const RotorPWM_<std::allocator<void>>>&);}
void PublishAsyncCommands(const ros::TimerEvent&);
}
'''
MOCK=DECLARATIONS+r'''
#include <atomic>
#include <thread>
#include <chrono>
std::atomic<int> active{},peak{};
void operation(){int n=++active;int p=peak.load();while(p<n&&!peak.compare_exchange_weak(p,n)){}
  std::this_thread::sleep_for(std::chrono::milliseconds(2));--active;}
void airsim_ros::commands_impl::ROTOR_PWM_Callback(const boost::shared_ptr<const RotorPWM_<std::allocator<void>>>&){operation();}
void airsim_ros::PublishAsyncCommands(const ros::TimerEvent&){operation();}
extern "C" int mock_peak(){return peak.load();}
'''
DRIVER=DECLARATIONS+r'''
#include <boost/shared_ptr.hpp>
#include <atomic>
#include <thread>
#include <cstdio>
namespace ros {struct TimerEvent{};}
extern "C" int mock_peak();
int main(){std::atomic<bool> start{false};
  std::thread a([&]{boost::shared_ptr<const airsim_ros::RotorPWM_<std::allocator<void>>> msg;
    while(!start.load()){};for(int i=0;i<32;++i)airsim_ros::commands_impl::ROTOR_PWM_Callback(msg);});
  std::thread b([&]{ros::TimerEvent event;while(!start.load()){};
    for(int i=0;i<32;++i)airsim_ros::PublishAsyncCommands(event);});
  start=true;a.join();b.join();std::printf("%d\n",mock_peak());}
'''


class BridgeProbeTests(unittest.TestCase):
    def test_observer_forwards_and_serializer_mutually_excludes_the_two_original_calls(self):
        root=Path(__file__).resolve().parents[2]
        build=subprocess.check_output(['bash',str(root/'tools/build_bridge_probe.sh')],cwd=root,text=True).strip()
        probe='/workspace/main/'+str(Path(build).relative_to(root))
        with tempfile.TemporaryDirectory(prefix='bridge_probe_test_',dir=root/'runs') as temporary:
            path=Path(temporary);inside='/workspace/main/'+str(path.relative_to(root))
            (path/'mock.cpp').write_text(MOCK);(path/'driver.cpp').write_text(DRIVER)
            def run(*args):
                return subprocess.check_output([str(root/'tools/enter_env.sh'),*args],cwd=root,text=True,stderr=subprocess.STDOUT)
            run('g++','-std=c++14','-shared','-fPIC','-pthread',inside+'/mock.cpp','-o',inside+'/mock.so')
            run('g++','-std=c++14','-pthread',inside+'/driver.cpp','-L'+inside,'-Wl,-rpath,'+inside,
                '-l:mock.so','-o',inside+'/driver')
            for mode,expected in (('observe',2),('serialize',1)):
                output=run('env','LD_PRELOAD='+probe,'RMUA_BRIDGE_PROBE_ORIGINAL='+inside+'/mock.so',
                    'RMUA_BRIDGE_PROBE_OUTPUT='+inside+'/'+mode+'.jsonl','RMUA_BRIDGE_PROBE_MODE='+mode,
                    inside+'/driver')
                self.assertEqual(int(output.strip()),expected)
                records=[json.loads(x) for x in (path/(mode+'.jsonl')).read_text().splitlines()]
                last=records[-1]
                self.assertEqual(last['event'],'exit')
                self.assertEqual((last['pwm_calls'],last['bridge_calls']),(32,32))
                self.assertEqual(last['write_errors'],0)
                self.assertEqual(last['overlap_entries']>0,mode=='observe')
                self.assertTrue(all(r['end_monotonic_ns']>=r['start_monotonic_ns'] for r in records))
                self.assertTrue(all(r['thread_id']!=r['other_thread_id'] and r['other_thread_id']>0
                                    for r in records if r['event']=='overlap'))


if __name__=='__main__':unittest.main()
