import unittest
import json
from evaluation.metrics.command_trace import evaluate,MD5


def record(seq,value):
    return {'md5sum':MD5,'data':{'header':{'seq':seq,'stamp':{'secs':10,'nsecs':seq*10000000}},
            **{'rotorPWM'+str(i):value for i in range(4)}}}


class CommandTraceTests(unittest.TestCase):
    def test_float64_wire_matches_but_sampled_debug_can_miss_a_real_command(self):
        telemetry=[{'command':{'sequence':i,'stamp':10+i*.01,'values':[value]*4},
                    'collective_pulse_applied':i==1} for i,value in enumerate((.2,.4,.2))]
        wire=[record(i,value) for i,value in enumerate((.2,.4,.2))]
        report=evaluate(telemetry,wire,[record(i,.2) for i in range(5)])
        self.assertTrue(report['wire_trace_valid'])
        self.assertEqual(report['experiment_pulses'][0]['debug_samples_matching_pulse'],0)
        json.dumps(report)
        self.assertFalse(report['actuator_consumption_verified'])
        rounded=[record(i,value) for i,value in enumerate((.20000000298023224,.4000000059604645,.20000000298023224))]
        self.assertFalse(evaluate(telemetry,rounded,[])['wire_trace_valid'])
        shifted=[record(i,value) for i,value in enumerate((.2,.4,.2))]
        for item in shifted:item['data']['header']['seq']+=1
        self.assertTrue(evaluate(telemetry,shifted,[])['wire_trace_valid'])
        self.assertEqual(evaluate(telemetry,shifted,[])['ros_header_sequence_offset'],1)
        self.assertFalse(evaluate(telemetry,wire[:-1],[])['wire_trace_valid'])
        wire[1]=record(1,.6)
        self.assertEqual(evaluate(telemetry,wire,[])['mismatched_sequences'],[1])


if __name__=='__main__':unittest.main()
