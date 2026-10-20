"""Read-only TCPROS header probe. Never constructs or sends a service request."""
import argparse
import json
from pathlib import Path


def observe():
    import roslib.message
    import rosservice
    name='/airsim_node/meter_report'
    uri=rosservice.get_service_uri(name)
    if not uri:
        raise RuntimeError('meter_report service not advertised')
    headers=rosservice.get_service_headers(name,uri)
    cls=roslib.message.get_service_class('airsim_ros/MeterReport')
    result={'service':name,'type':headers.get('type'),'server_md5':headers.get('md5sum'),
            'local_md5':cls._md5sum if cls else None,'headers':headers,
            'service_request_sent':False,'business_success_semantics_verified':False,
            'formal_server_permissions_verified':False}
    result['compatible']=(cls is not None and result['type']=='airsim_ros/MeterReport'
                           and result['server_md5']==cls._md5sum=='40e1aa17dd96ff6a8c88396581d6a491')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--output',type=Path)
    group.add_argument('--run-dir',type=Path)
    args=parser.parse_args()
    result=observe()
    output=args.output if args.output else args.run_dir/'service_contract_audit.json'
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['compatible'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
