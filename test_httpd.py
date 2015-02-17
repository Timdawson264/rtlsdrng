#!/usr/bin/python2

#
# pip install tinyrpc
#

from tinyrpc.protocols.jsonrpc import JSONRPCProtocol
from tinyrpc.transports.http import HttpPostClientTransport
from tinyrpc import RPCClient

rpc_client = RPCClient(
    JSONRPCProtocol(),
    HttpPostClientTransport('http://localhost:8888/json/')
)

local = rpc_client.get_proxy()

add_freq = local.add_freq(freq='12e6',mod='fm',squalch='50',gain='auto', 
device='0' )

test = local.test(a='1')

print 'rpc test done'

import sys
sys.exit(0)
